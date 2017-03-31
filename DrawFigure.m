function DrawFigure()
DataHys = load('d:\\hysdata.txt');
TimeHys = DataHys(:,1);
CurrentHys = DataHys(:,2);
SpeedHys = DataHys(:,3);
IStateHys = DataHys(:,4);
%SStateHys = DataHys(:,5);
IGBTHys = DataHys(:,6);

DataPID = load('d:\\piddata.txt');
TimePID = DataPID(:,1);
CurrentPID = DataPID(:,2);
SpeedPID = DataPID(:,3);
IStatePID = DataPID(:,4);
%SStatePID = DataPID(:,5);
IGBTPID = DataPID(:,6);

figure(1);
subplot(2,1,1);
%[AX,~,~] = plotyy(TimePID,CurrentPID,TimePID,IStatePID,'plot');
%[AX,~,~] = plotyy(TimeHys,CurrentHys,TimeHys,IStateHys,'plot');
[AX,~,~] = plotyy(TimeHys(35000:45000),CurrentHys(35000:45000),TimeHys(35000:45000),IStateHys(35000:45000),'plot');
%[AX,~,~] = plotyy(TimePID(15000:25000),CurrentPID(15000:25000),TimePID(15000:25000),IStatePID(15000:25000),'plot');
title('Q-Winding Current');
%axis(AX(1),[-Inf,Inf,-Inf,16]);
axis(AX(1),[0.35,0.45,-Inf,16]);
%axis(AX(2),[-Inf,Inf,0,5]);
axis(AX(2),[0.35,0.45,0,5]);
set(AX(1),'ytick',[0,14,15]);
set(AX(2),'ytick',[0,1]);
xlabel('Time / s');
ylabel(AX(1),'Iq / A');
ylabel(AX(2),'Current Controller');
hold on;
%plot(TimePID,SpeedPID);
%plot(TimeHys,CurrentHys);
plot([0,TimeHys(end)],[15,15],'Color',[180 180 180]/255);
plot([0,TimeHys(end)],[14,14],'Color',[180 180 180]/255);
%hold off;
%legend('PID Controller','Hysteresis Controller');

subplot(2,1,2);
%[AX,~,~] = plotyy(TimePID,SpeedPID,TimePID,IGBTPID,'plot');
%[AX,~,~] = plotyy(TimeHys,SpeedHys,TimeHys,IGBTHys,'plot');
[AX,~,~] = plotyy(TimeHys(35000:45000),SpeedHys(35000:45000),TimeHys(35000:45000),IStateHys(35000:45000),'plot');
%[AX,~,~] = plotyy(TimePID(15000:25000),SpeedPID(15000:25000),TimePID(15000:25000),IGBTPID(15000:25000),'plot');
title('Rotating Speed');
%axis(AX(1),[-Inf,Inf,0,140]);
axis(AX(1),[0.35,0.45,0,140]);
%axis(AX(2),[-Inf,Inf,0,5]);
axis(AX(2),[0.35,0.45,0,5]);
set(AX(1),'ytick',[0,80,120]);
set(AX(2),'ytick',[0,1]);
xlabel('Time / s');
ylabel(AX(1),'Omega / (rad/s)');
ylabel(AX(2),'Controller');
hold on;
%plot(TimePID,SpeedPID);
%plot(TimeHys,SpeedHys);
plot([0,TimeHys(end)],[80,80],'Color',[180 180 180]/255);
 plot([0,TimeHys(end)],[82,82],'Color',[180 180 180]/255);
 plot([0,TimeHys(end)],[78,78],'Color',[180 180 180]/255);
plot([0,TimeHys(end)],[120,120],'Color',[180 180 180]/255);
 plot([0,TimeHys(end)],[122,122],'Color',[180 180 180]/255);
 plot([0,TimeHys(end)],[118,118],'Color',[180 180 180]/255);
%hold off;
end