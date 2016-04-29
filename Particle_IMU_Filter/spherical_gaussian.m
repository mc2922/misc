function spherical_gaussian

look_azimuths = linspace(0,360,360)*pi/180;
look_elevations = linspace(0,180,180)*pi/180;
lobe_azimuth = 58.715*pi/180;
lobe_elevation = 118.305*pi/180;
lobe_vector = [sin(lobe_elevation)*cos(lobe_azimuth), sin(lobe_elevation)*sin(lobe_azimuth), cos(lobe_elevation)];
lobe_sharpness = 10;
lobe_amplitude = 2;
lobe_values = zeros(length(look_elevations), length(look_azimuths));

for elev_idx = 1:length(look_elevations)
    for azim_idx = 1:length(look_azimuths)
        look_vector = [sin(look_elevations(elev_idx))*cos(look_azimuths(azim_idx)), sin(look_elevations(elev_idx))*sin(look_azimuths(azim_idx)), cos(look_elevations(elev_idx))];
        lobe_values(elev_idx, azim_idx) = lobe_amplitude*exp(lobe_sharpness*(lobe_vector*look_vector' - 1));
    end
end

min_lobe = min(lobe_values(:));
max_lobe = max(lobe_values(:));
lobe_values = (lobe_values-min_lobe)/(max_lobe-min_lobe);
% lobe_values = lobe_values*(lobe_amplitude-1) + 1;

% x_space = zeros(length(look_elevations), length(look_azimuths));
% y_space = zeros(length(look_elevations), length(look_azimuths));
% z_space = zeros(length(look_elevations), length(look_azimuths));
% for elev_idx = 1:length(look_elevations);
%     for azim_idx = 1:length(look_azimuths);
%         x_space(elev_idx, azim_idx) = sin(look_elevations(elev_idx))*cos(look_azimuths(azim_idx));
%         y_space(elev_idx, azim_idx) = sin(look_elevations(elev_idx))*sin(look_azimuths(azim_idx));
%         z_space(elev_idx, azim_idx) = cos(look_elevations(elev_idx));
%     end
% end
% 
% h2 = surf(x_space,y_space,z_space,lobe_values);
% set(h2,'EdgeColor', 'none');
% axis equal;
% view(135,30)

figure;
h = pcolor(look_azimuths, look_elevations, lobe_values);
set(h,'EdgeColor', 'none');
axis equal;

lobe_values_x = zeros(length(look_elevations), length(look_azimuths));
lobe_values_y = zeros(length(look_elevations), length(look_azimuths));
lobe_values_z = zeros(length(look_elevations), length(look_azimuths));
for elev_idx = 1:length(look_elevations)
    for azim_idx = 1:length(look_azimuths)
        lobe_values_x(elev_idx, azim_idx) = sin(look_elevations(elev_idx))*cos(look_azimuths(azim_idx));
        lobe_values_y(elev_idx, azim_idx) = sin(look_elevations(elev_idx))*sin(look_azimuths(azim_idx));
        lobe_values_z(elev_idx, azim_idx) = cos(look_elevations(elev_idx));
    end
end
lobe_values = lobe_values_x*lobe_vector(1) + lobe_values_y*lobe_vector(2) + lobe_values_z*lobe_vector(3);
lobe_values = lobe_amplitude*exp(lobe_sharpness*(lobe_values-1));
figure;
h = pcolor(look_azimuths, look_elevations, lobe_values);
set(h,'EdgeColor', 'none');
axis equal;

end