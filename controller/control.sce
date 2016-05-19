function control(speed)
M = csvRead(strcat(["CSV/control_",string(speed),".csv"]))
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
n = size(Omega)-1
hz = iir(2,'lp','butt',[2/Fs],[])
Omega_out = flts(Omega,hz)
A = 1
q_est = Omega_out($)
k_est = q_est/A

y_0 = Omega_out(1)
y_inf = q_est
y_max = max(Omega_out) 
Overshoot = abs(y_max - y_inf)/abs(y_0 - y_inf)
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
scf(speed);
clf;
plot(t',Omega_out);
plot([Ts, Ts], [0, Omega_at_Ts], 'k');
Over = (max(Omega_out) - Omega_out($))/Omega_out($)
printf("control= %d, speed= %f, overshoot= %f Ts= %d\n",speed, Omega_out($),Over, Ts);
endfunction

for j = 3:1:14
    control(j)
end
