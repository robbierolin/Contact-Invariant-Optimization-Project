function [ ret ] = q( s,N,T,K,D)
%Q Return a vector of 3d coordinates the represent character pose.

ret = zeros(T,D);
[tp,to,lfp,lfo,rfp,rfo,lhp,lho,rhp,rho] = getBodyPositions( s, K, N );
tp_spline = cscvn(tp);
to_spline = cscvn(to);
lfp_spline = cscvn(lfp);
lfo_spline = cscvn(lfo);
rfp_spline = cscvn(rfp);
rfo_spline = cscvn(rfo);
lhp_spline = cscvn(lhp);
lho_spline = cscvn(lho);
rhp_spline = cscvn(rhp);
rho_spline = cscvn(rho);

for t=1:T
   ret(t, 1:3) = fnval(tp_spline, ((K*t/T)-1)*tp_spline.breaks(2)); 
   ret(t, 4:6) = fnval(lfp_spline, ((K*t/T)-1)*lfp_spline.breaks(2)); 
   ret(t, 7:9) = fnval(rfp_spline, ((K*t/T)-1)*rfp_spline.breaks(2));
   ret(t, 10:12) = fnval(lhp_spline, ((K*t/T)-1)*lhp_spline.breaks(2)); 
   ret(t, 13:15) = fnval(rhp_spline, ((K*t/T)-1)*rhp_spline.breaks(2)); 
   ret(t, 16:18) = fnval(to_spline, ((K*t/T)-1)*to_spline.breaks(2)); 
   ret(t, 19:21) = fnval(lfo_spline, ((K*t/T)-1)*lfo_spline.breaks(2)); 
   ret(t, 22:24) = fnval(rfo_spline, ((K*t/T)-1)*rfo_spline.breaks(2)); 
   ret(t, 25:27) = fnval(lho_spline, ((K*t/T)-1)*lho_spline.breaks(2)); 
   ret(t, 28:30) = fnval(rho_spline, ((K*t/T)-1)*rho_spline.breaks(2)); 
end

end

