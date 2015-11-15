%% Nonlinear model for surge, page 141 in the handbook
function e = odefun(P, udata, tdata, nc)

dt = tdata(2) - tdata(1);

u(1) = udata(1);
e(1) = u(1) - udata(1);

for i=1:length(tdata)-1
   u(i+1) = u(i) + (dt/P(1))*(nc + P(2)*u(i)*abs(u(i)) + P(3)*u(i));
   e(i+1) = u(i+1) - udata(i+1);
end

end