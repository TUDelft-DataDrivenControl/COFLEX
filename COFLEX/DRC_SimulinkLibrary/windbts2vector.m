function [U, T, DT] = windbts2vector(fileName)
[velocity, twrVelocity, y, z, zTwr, nz, ny, dz, dy, dt, zHub, z1,mffws] = readfile_BTS(fileName);

[m,n,o,p] = size(velocity);
U = zeros(1,m);
for ii = 1:m
    umean = squeeze(velocity(ii,1,:,:));
    umean = mean(umean(:));
    U(ii) = umean;
end

T = 0:dt:(length(U)-1)*dt;
DT = round(dt*1e4)/1e4;

end

