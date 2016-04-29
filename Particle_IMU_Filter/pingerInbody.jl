

Ry(th) = [[cos(th);0.0;sin(th)]';[0.0;1.0;0.0]';[-sin(th);0.0;cos(th)]']'
Rz(th) = [[cos(th);sin(th);0.0]';[-sin(th);cos(th);.0]';[0.0;0.0;1.0]']'

bRp(az,th) = Rz(az)*Ry(th)
bRpe(az::Float64,el::Float64) = bRp(az, pi/2-el )
bRpe(pt::Array{Float64,1}) = bRpe(pt[1],pt[2])
peRb(pt::Array{Float64,1}) = atan2(pt[2],pt[1]), atan2(norm(pt[1:2]),pt[3]) # az, el

function bRa(acc=[0.0;0.0;9.81],mag=[1.0;0.0;0.0])
  g = acc/norm(acc)
  m = mag/norm(mag)
  v = cross(g,m)
  w = cross(v,g)
  return [w'/norm(w); v'/norm(v); g']'
end

function wrapRad(th::Float64)
  if th >= pi
    th -= 2.0*pi
  end
  if th < -pi
    th += 2.0*pi
  end
  return th
end

function getlRb(x::State)
  # ref = [1.0;0.0;0.0]
  nIa =  norm(x.lasta) # from accelerometers b[xyz] -- fwd-prt-up
  nIm =  1.0

  lRb = Rz(x.yaw)*bRa(x.lasta)
  return lRb
end

function propNavIMU(w,a,yaw=0.0)
  # fwd-prt-up
  bRaVAL = bRa(a)
  aRb = bRaVAL'
  aGyr = aRb*w # w = bodyGyro
  dAzi = aGyr[3]
  yaw = yaw + dAzi * dt # integrate yaw
  yaw = wrapRad(yaw)
  lRb = Rz(yaw)*aRb
  return lRb, yaw
end

# not totally considering World Magnetic Model yet
function magUpdateYaw!(x::State, mag::Array{Float64,1}; mRef::Array{Float64,1}=[1.0;0.0;0.0])
  lMag = getlRb(x)*mag
  magYaw = atan2(lMag[2],lMag[1])
  dyaw = magYaw - yaw
  filtdyaw = filter(dyaw)
  x.yaw = x.yaw + filtdyaw
  nothing
end

function projPtsPlaneToBall!(pts::Array{Float64,2}, bpts::Array{Float64,2}=zeros(3,0))
  cols = size(pts,2)
  if cols != size(bpts,2)  bpts = zeros(3,cols);  end
  for i in 1:size(pts,2)
    bpts[:,i] = bRpe(vec(pts[:,i]))
  end
  nothing
end

type State
  yaw::Float64
  lParticles::Array{Float64,2} # stored in local
  lasta::Array{Float64,1}
end

function upStateIMU!(x::State, w::Array{Float64,1}, a::Array{Float64,1})
  lRb, x.yaw = propNavIMU(w, a, x.yaw)
  x.lasta = deepcopy(a)
  nothing
end

function upStateAcoustics!(x::State, N::Int, wGrid::Array{Float64,2})
  #importance sampling step to incorporate beam forming measurement information
  lRb = getlRb(x)
  # all particles in body ball
  bRl = lRb'
  azel = zeros(2,N)
  for i in 1:N
    azel[:,i] = peRb(vec(bRl*x.lParticles[:,i]))
  end
  # sample importance

  #resample (equal weight) -- decrease in inter-particle correlation

  # go back to local ball
  for i in 1:N
    x.lParticles[:,i] = lRb*bRpe(vec(azel[:,i]))
  end
  nothing
end

function runEstimator(data; N=500)
  # init state
  initpts = [2.0*pi*rand(1,N)';pi*rand(1,N)'] # az; el
  bpts = zeros(3,N)
  projPtsPlaneToBall!(initpts, bpts)
  X = State(0.0, bpts, zeros(3))

  @sync begin
    @async begin
      while false
        # get next data line
          # (w,a), wGrid
        # if imu
          # upStateIMU!(X, w, a)
        # end
        # if magmeas
        #   magUpdateYaw!(x,mag)
        # end
        # if acoustics
          # upStateAcoustics(X,N,wGrid)
        # end
      end
    end
    @async begin
      # sevice user
      X
    end
  end

end



# tests
# julia> bRpe(0,pi/2)*[1.0;0.0;0.0]
# 3-element Array{Float64,1}:
#  1.0
#  0.0
#  0.0
#
# julia> bRpe(0,pi/4)*[1.0;0.0;0.0]
# 3-element Array{Float64,1}:
#  0.707107
#  0.0
#  0.707107
#
# julia> bRpe(pi/2,pi/4)*[1.0;0.0;0.0]
# 3-element Array{Float64,1}:
#  4.32978e-17
#  0.707107
#  0.707107
#
# julia> bRpe(pi,pi/4)*[1.0;0.0;0.0]
# 3-element Array{Float64,1}:
#  -0.707107
#   8.65956e-17
#   0.707107
#
#
#   julia> bRp(0.0,0.0)*[1.0;0.0;0.0]
# 3-element Array{Float64,1}:
#  1.0
#  0.0
#  0.0
#
# julia> bRp(pi/2,0.0)*[1.0;0.0;0.0]
# 3-element Array{Float64,1}:
#  6.12323e-17
#  1.0
#  0.0
#
# julia> bRp(pi/2,pi/4)*[1.0;0.0;0.0]
# 3-element Array{Float64,1}:
#  4.32978e-17
#  0.707107
#  0.707107
#
# julia> bRp(-pi/2,pi/4)*[1.0;0.0;0.0]
# 3-element Array{Float64,1}:
#   4.32978e-17
#  -0.707107
#   0.707107
#
# julia> bRp(pi/2,-pi/4)*[1.0;0.0;0.0]
# 3-element Array{Float64,1}:
#   4.32978e-17
#   0.707107
#  -0.707107
#
# julia> bRp(pi,-pi/4)*[1.0;0.0;0.0]
# 3-element Array{Float64,1}:
#  -0.707107
#   8.65956e-17
#  -0.707107
