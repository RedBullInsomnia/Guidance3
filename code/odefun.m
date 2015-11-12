function e = odefun(p, rdata, tdata,dc)
%% Discretized nomoto model

% p(1) = T1
% p(2) = T2
% p(3) = K


bech = @(x) 1872000*x.^3 -27.31*x;

u = -dc;
dt = tdata(2) - tdata(1);
r(1) = rdata(1);
%r(2) = -dt^2/(-2*p(1)*p(2) + (p(1) + p(2))*dt)*p(3)*u;
r(2) = rdata(2);
e(1) = r(1) - rdata(1);
e(2) = r(2) - rdata(2);
for i=1:length(tdata)-2
   r(i+2) = dt^2/(p(1)*p(2))*(p(3)*u - p(3)*bech(r(i)) - (p(1)+p(2))/dt*(r(i+1) - r(i))) + 2*r(i+1) - r(i);
   e(i+2) = r(i+2) - rdata(i+2);
end

r
end

