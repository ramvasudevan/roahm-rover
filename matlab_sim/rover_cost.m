function expr = rover_cost(k_0,k_1,Zg_0,Zg_1)
%ROVER_COST
%    EXPR = ROVER_COST(K_0,K_1,ZG_0,ZG_1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    21-Jan-2020 14:57:48

t2 = k_0.*(2.89e2./1.0e2);
t3 = t2+2.89e2./1.0e2;
t4 = k_0.*1.012110726643599e-1;
t5 = t4+1.012110726643599e-1;
t6 = k_1.*t3.*t5.*5.376838235294118e-2;
t7 = k_0.*5.060553633217993e-2;
t8 = t7+5.060553633217993e-2;
t9 = k_1.*t3.*t8.*5.376838235294118e-2;
t10 = k_1.^2;
t11 = t3.^2;
t12 = k_0.*1.518166089965398e-1;
t16 = t8.*t10.*t11.*2.891038940852076e-3;
t13 = t12-t16+1.518166089965398e-1;
t14 = k_1.*t3.*t13.*5.376838235294118e-2;
t15 = t6+t9;
t17 = k_0.*(1.17e2./5.78e2);
t18 = k_1.*t3.*t15.*5.376838235294118e-2;
t19 = t6+t9+t14;
t20 = k_1.*t3.*t19.*5.376838235294118e-2;
t24 = k_0.*2.530276816608997e-1;
t21 = t16+t18+t20-t24-2.530276816608997e-1;
t22 = t16-t17+t18-1.17e2./5.78e2;
t25 = k_1.*t3.*t21.*5.376838235294118e-2;
t26 = k_0.*3.036332179930796e-1;
t27 = k_1.*t3.*t22.*5.376838235294118e-2;
t28 = t6+t9+t14-t27;
t29 = k_1.*t3.*t28.*5.376838235294118e-2;
t30 = t16+t18+t20-t26+t29-3.036332179930796e-1;
t31 = k_1.*t3.*t30.*5.376838235294118e-2;
t32 = t6+t9+t14-t25-t27;
t33 = k_1.*t3.*t32.*5.376838235294118e-2;
t23 = Zg_0-k_0.*(1.17e2./2.89e2)+t16+t18+t20+t33+k_1.*t3.*(t6+t9+t14-t25-t27-t31).*5.376838235294118e-2+k_1.*t3.*(t6+t9+t14+k_1.*t3.*(t17-k_1.*t3.*t15.*5.376838235294118e-2-t8.*t10.*t11.*2.891038940852076e-3+1.17e2./5.78e2).*5.376838235294118e-2).*5.376838235294118e-2-1.17e2./2.89e2;
t34 = Zg_1-t6-t9-t14+t25+t31+k_1.*t3.*(t16-t17+t18-1.17e2./5.78e2).*5.376838235294118e-2+k_1.*t3.*(k_0.*(-3.542387543252595e-1)+t16+t18+t20+t29+t33-3.542387543252595e-1).*5.376838235294118e-2;
expr = t23.^2+t34.^2;