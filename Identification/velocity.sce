function identification(data)
M = csvRead(strcat(["CSV/5m_P",string(data),".csv"]))
initcount = 0 
initms = 0
prescount = 0
presms = 0
Omega = zeros(size(M)-1)
presOmega = 0
calcOmega = 0
t = M(2:$,2)
for i = 2:(size(M,1)-1)
    prescount = M(i,3)
    presms = M(i,2)
    if initcount < prescount then 
        calcOmega = ((prescount - initcount)*%pi*1000)/((presms-initms)*180)
        initms = presms
        initcount = prescount
        presOmega = calcOmega
    end
    Omega(i) = presOmega
end

Fs = 200
hz = iir(2,'lp','butt',[1.2/Fs],[])
Omega_out = flts(Omega,hz)
A = data
q_est = Omega_out($)
k_est = q_est/A

y_0 = Omega_out(1)
y_inf = q_est
y_max = max(Omega_out) 
Overshoot = abs(y_max - y_inf)/abs(y_0 - y_inf)
xi_est = sqrt(log(Overshoot)^2/(%pi^2 + log(Overshoot)^2))

alpha = 5
Ts = -1
Omega_at_Ts = -1
for i = 250:-1:1
    if abs(q_est - Omega_out(i)) < alpha/100*q_est
        Ts = t(i)
        Omega_at_Ts = Omega_out(i)
    else 
        break
    end
end

if Ts == -1 then
    disp('No settling time available')
end

N_bar = 1/sqrt(1 - xi_est^2)
o_n_est = (log(alpha/100) - log(N_bar))/(-xi_est*Ts)*1000

// Simulation
time = 0:0.001:size(Omega,2)/Fs
s = poly(0, 's');
G_est = k_est/(s^2/o_n_est^2 + 2*xi_est/o_n_est*s + 1);
G_est = syslin('c', G_est);
Omega_est = csim(ones(1,size(time,2))*A, time, G_est);

// Plot
scf(data);
clf;
plot(t,Omega_out');
plot([Ts, Ts], [0, Omega_at_Ts], 'k');
plot(Omega_est, 'g');
printf("data= %d, k_est= %f, o_n= %f xi= %f\n",data,k_est,o_n_est,xi_est);
// Error Calculation 
ISE = 0
IAE = 0
ITSE = 0
ITAE = 0 
for i = 1:size(t,1)
      E(i) = Omega_est(i) - Omega(i)
      ISE = ISE + (E(i)^2) 
       IAE = IAE + abs(E(i))
       ITSE = ITSE + i*0.005*(E(i)^2)
       ITAE = ITAE + i*0.005*abs(E(i))
end    
printf("ISE= %d, IAE= %f, ITSE= %f ITAE= %f\n",ISE,IAE,ITSE,ITAE);  
printf("%d",Ts)
endfunction

for j = 20:10:100
    identification(j)
end
