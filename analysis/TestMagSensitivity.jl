using Gadfly


north(p,alp) = p*cos(alp)
west(p,alp) = p*sin(alp)

psi = 45.0
d2r = pi/180.0

m=100000
P = 100+3.0*randn(m)
A = (10.0*randn(m)+psi)*d2r

N, W = Float64[], Float64[]

for i in 1:m
  push!(N, north(P[i],A[i]))
  push!(W, west(P[i],A[i]))
end

plot(x=-W,y=N,Geom.histogram2d())
