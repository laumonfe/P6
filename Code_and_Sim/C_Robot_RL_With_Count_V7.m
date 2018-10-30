%   This code is inspired by Matthew Sheen, 2015 and is avalable here:
%   "https://se.mathworks.com/matlabcentral/fileexchange/57882-reinforcement-learning-example-pendulum-controller-w--animation"
%   The code have been made to solve your project but some code is still
%   there from the original code, credits go the Matthew Sheen for these parts of the code.

function C_Robot_RL_With_Count_V7()
disp('Program started');
%For the connection to V-rep
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%For Excel to save the data
Counter = 0;
sheet = 1;

%To save the information from V-rep and to V-rep        %Plz explain
Pcount(1:4) = 1;%Pick up counter
Pcount(4) = 0;%Pick up counter      % Do we resent it here? 
Pcount_temp(1:4) = 0;%Pick up counter
Pcount_Stop(1:4) = 0;%Stop counter
Pcount_Stop(4) = 1;%Stop counter

%For calculating stuff for the reinforcement
goalPt = [(1/3)+1,(1/3)+1,(1/3)+1]; %The goal point for the agent.
distanceGoal = @(x,y)sqrt((x-goalPt(1)).^2+(y-goalPt(2)).^2+((4-(x+y))-goalPt(3)).^2); %The distance to the goal from where we are. % Explain plz
rewardFuncR = @(x)2.^-(5*x-6);%Reward func
TT = (Pcount(1) + Pcount(2) + Pcount(3)); %Total amount of blobs picked up.
test = [round((Pcount(1)/TT)+1,2),round((Pcount(2)/TT)+1,2),round((Pcount(3)/TT)+1,2)];%Picked up in procent % what is the 2 after the comma?

%Values for the reinforcement learning.
learnRate = 0.99;
epsilon = 0.50;
epsilonDecay = 0.98;
discount = 0.9;% what does this mean? discount factor
successRate = 1; %what is this?
actionsR = [0,1,0,1];%Action for the  robot

%All the different combination of action for all robots % explain
Rob1 = 1:0.01:2;
Rob2 = 1:0.01:2;
Rob3 = -1.5:0.1:0;

%Clobal values for the program
Start = 1;
OsIdx = 1; %what?
OaIdx = 1; %what?
lostBox = 0;
lostBox_temp = 0; %what?
RobotWork = 0; %what?
p_temp = 0; %what?
aIdx_temp = 0; %what?
minus1 = 0; %what?
Flag = 0; %what?
Pcount_save = 0;

%-------------------- Is this the initialization?
DoSave = 0;
RunPhase1 = 1; 
Phase1 = 1000;
Phase2 = 50;
Steps = 250;
%--------------------

%Create an array of all the combination of actions for the robots %explain
statesR=zeros(length(Rob1)*length(Rob2)*length(Rob3),3);
indexR=1;
for j = 1:length(Rob1)
    for h = 1:length(Rob2)
        for i = 1:length(Rob3)
            statesR(indexR,1)=Rob1(j);
            statesR(indexR,2)=Rob2(h);
            statesR(indexR,3)=Rob3(i);
            indexR=indexR+1;
        end
    end
end
DGR = distanceGoal(statesR(:,1),statesR(:,2)); %Distance to the goal from each state.
RR = rewardFuncR(DGR(:,:));%Reward from each state.
if DoSave == 0 || DoSave == 1
    QR = xlsread('testdata3.xlsx');% excel file? 
    if QR == 0
        QR = repmat(RR,[1,4]); %replicate matrix? 
    end
else
    QR = repmat(RR,[1,4]); %Reward from each action.
end %why is it doing the same regardless? 
%-----------------------Init-------------------------
%For fast learning so we dont have to wait for the sim.
tic %Timer start
while (Start <= Phase1 && RunPhase1 == 0 && clientID>-1 && vrep.simxGetConnectionId(clientID)~=-11)
    Pcount(1:3) = 1;
    TT = 0;
    %Pcount_Stop(1:2) = 0;
    bonus = 0;
    fprintf("-------------------------------------------\n");
    while (TT <= Steps)
        if clientID>-1 && vrep.simxGetConnectionId(clientID)~=-1
            TT = (Pcount(1) + Pcount(2) + Pcount(3)); %Total amount of blobs picked up.
            test = [round((Pcount(1)/TT)+1,2),round((Pcount(2)/TT)+1,2),round((Pcount(3)/TT)+1,2)];%Picked up in procent %
            [~,sIdx] = min(sum((statesR - repmat(test,[size(statesR,1),1])).^2,2));%Find what state is closest to the state we are in. %math?
            if (rand()>epsilon || TT == Steps) && rand()<=successRate%Greedy policy % why random?
                [~,aIdx] = max(QR(sIdx,:)); %Best action. %explain
            else%non-Greedy policy
                aIdx = randi(length(actionsR),1); %Random action.
            end
            %Do the  action
            if aIdx == 1
                Pcount_Stop(1) = 0;%Stop robot
                Pcount_Stop(2) = 0;%Stop robot
            elseif aIdx == 2
                Pcount_Stop(1) = 0;%Stop robot
                Pcount_Stop(2) = 1;%Start robot
            elseif aIdx == 3
                Pcount_Stop(1) = 1;%Start robot
                Pcount_Stop(2) = 0;%Stop robot
            elseif aIdx == 4
                Pcount_Stop(1) = 1;%Start robot
                Pcount_Stop(2) = 1;%Start robot
            end
        % what is this?
            if Pcount_Stop(1) == 0 && Pcount_Stop(2) == 0
                Pcount(1) = (Pcount(1) + 1);%Stop or start the robot
                Pcount(2) = (Pcount(2) + 1);%Stop or start the robot
                RobotWork = 4;
            elseif Pcount_Stop(1) == 0
                Pcount(1) = (Pcount(1) + 1);%Stop or start the robot
                RobotWork = 1;
            elseif Pcount_Stop(2) == 0
                Pcount(2) = (Pcount(2) + 1);%Stop or start the robot
                RobotWork = 1;
            else
                %Do nothing == Robot stopped
            end
            
            if rand()>=(0+0.20*RobotWork) %what?
                Pcount(3) = (Pcount(3) + 1);%Stop or start the robot isn't robot 3 always working?
            end
            
            if ( 1.31 <= test(1) && test(1) <= 1.35)
                if ( 1.31 <= test(2) && test(2) <= 1.35)
                    if ( 1.31 <= test(3) && test(3) <= 1.35)
                        if (TT > 3) % what is this? 
                            Start = Start+1;
                            if mod(Start,1) == 0
                                disp(test)
                                fprintf("-------------------------------------------\n");
                                fprintf("YEAH!!!!!  \n");
                                disp(Start);
                                fprintf("-------------------------------------------\n");
                            end
                            bonus = 90;
                            if  mod(Start,100) == 0 && DoSave == 0
                                %For Ecxel
                                filename = 'testdata3.xlsx';
                                Save = QR;
                                xlRange = num2str(1);
                                xlswrite(filename,Save,sheet,xlRange)
                            end
                        end
                    end
                end
            end
            
            epsilon = epsilon*epsilonDecay; %Probability
            
            if mod(TT,5) == 0 %what?
                disp(test)
            end
            
            if ((test(1) >= goalPt(1)+0.05) || (test(2) >= goalPt(2)+0.05) || (test(3) >= goalPt(3)+0.05))
                bonus = bonus-15*DGR(sIdx); %what?
            elseif ((test(1) <= goalPt(1)-0.05) || (test(2) <= goalPt(2)-0.05) || (test(3) <= goalPt(3)-0.05))
                bonus = bonus-15*DGR(sIdx);
            else
            end
            
            for p_temp = -1.5:0.1:0 %what?
                Compare = [test(1),test(2),p_temp];
                [~,snewIdx] = min(sum((statesR - repmat(Compare,[size(statesR,1),1])).^2,2));
                QR(sIdx,aIdx) = (1-learnRate)*QR(sIdx,aIdx) + learnRate * (RR(snewIdx) + discount*max(QR(snewIdx,:)) - QR(sIdx,aIdx) + bonus );
            end
            
            %             if success == true || success
            %                 break;
            %             end
        end
    end
end
toc%Timer end
if DoSave == 0
    %For Ecxel
    filename = 'testdata3.xlsx';
    Save = QR;
    xlRange = num2str(1);
    xlswrite(filename,Save,sheet,xlRange)
end
%Reset Eps
epsilon = 0.01;
%-----------------------V-rep-------------------------
while (clientID>-1)
    Start = 1;
    while (Start <= Phase2 && clientID>-1 && vrep.simxGetConnectionId(clientID)~=-1)
        Want1 = 0;%Reset
        Pcount_temp(1:4) = 0;
        %Pcount_Stop(1:2) = 0;
        %tic %Timer start;
        bonus = 0;
        %Connection to the v-rep for the pickup count.
        while Want1 == 0 && clientID>-1 && vrep.simxGetConnectionId(clientID)~=-1
            [err,signal]=vrep.simxReadStringStream(clientID,'PcountABC',vrep.simx_opmode_streaming);
            [err,signal]=vrep.simxReadStringStream(clientID,'PcountABC',vrep.simx_opmode_buffer);
            if (err==vrep.simx_return_ok)
                Pcount_temp2(1:4) = vrep.simxUnpackFloats(signal);
                Pcount(1:4) = (vrep.simxUnpackFloats(signal)-Pcount_temp2);
                TT = (Pcount(1) + Pcount(2) + Pcount(3)); %Total amount of blobs picked up.
                test = [round((Pcount(1)/TT)+1,2),round((Pcount(2)/TT)+1,2),round((Pcount(3)/TT)+1,2)];
                Want1 = 1;
            end
        end
        while TT <= Steps
            Want1 = 0;%Reset
            Want2 = 0;%Reset
            %Connection to the v-rep for the pickup count.
            while Want1 == 0 && clientID>-1
                [err,signal]=vrep.simxReadStringStream(clientID,'PcountABC',vrep.simx_opmode_streaming);
                [err,signal]=vrep.simxReadStringStream(clientID,'PcountABC',vrep.simx_opmode_buffer);
                if (err==vrep.simx_return_ok)
                    PcountDGR = vrep.simxUnpackFloats(signal);
                    Pcount(1:4) = (vrep.simxUnpackFloats(signal)-Pcount_temp2);
                    TT = (Pcount(1) + Pcount(2) + Pcount(3)); %Total amount of blobs picked up.
                    
                    if TT >= (Steps/2)-1 && TT <= (Steps/2)+1
                        Pcount_save = Pcount(1:3);
                    elseif TT >= Steps-1 && TT <= Steps+1
                        Pcount_temp2(1:3) = Pcount_temp2(1:3)+Pcount_save;
                        Pcount_temp(1:3) = Pcount_temp(1:3) - Pcount_save;
                    end
                    
                    Pcount(1:4) = (vrep.simxUnpackFloats(signal)-Pcount_temp2);
                    TT = (Pcount(1) + Pcount(2) + Pcount(3)); %Total amount of blobs picked up.
                    test = [round((Pcount(1)/TT)+1,2),round((Pcount(2)/TT)+1,2),round((Pcount(3)/TT)+1,2)];
                    Want1 = 1;
                end
            end
            %Connection to the v-rep for the box info.
            while Want2 == 0 && clientID>-1
                [err,signal]=vrep.simxReadStringStream(clientID,'Pakke',vrep.simx_opmode_streaming);
                [err,signal]=vrep.simxReadStringStream(clientID,'Pakke',vrep.simx_opmode_buffer);
                if (err==vrep.simx_return_ok)
                    Pakke(1:51) = vrep.simxUnpackFloats(signal);
                    Want2 = 1;
                    try
                        Pakke(51);
                    end
                end
            end
            
            Box = Pakke(51)-1;
            if (p_temp > Pakke(Box*5+1) && minus1 ~= 0)
                minus1 = minus1 - 1;
            elseif ((Pakke(1+(Box-minus1)*5+2) == 0 || Pakke(1+(Box-minus1)*5+3) == 0 || Pakke(1+(Box-minus1)*5+4) == 0) && (Pakke(1+(Box-minus1)*5) <= -1.5))
                minus1 = minus1 + 1;
                lostBox = lostBox+1;
            elseif (Pakke(1+(Box-minus1)*5) <= -1.5)
                minus1 = minus1 + 1;
            end
            
            if ((Pakke(1+(Box-minus1)*5+2) == 0 || Pakke(1+(Box-minus1)*5+3) == 0 || Pakke(1+(Box-minus1)*5+4) == 0) && (Pakke(1+(Box-minus1)*5) <= -1.5))
                minus1 = minus1 + 1;
                lostBox = lostBox+1;
            elseif (Pakke(1+(Box-minus1)*5) <= -1.5)
                minus1 = minus1 + 1;
            end
            
            p_temp = Pakke(Box*5+1);
            dist = (round(Pakke((Box-minus1)*5+1)*10))/10;
            
            %Check if we lose a blob
            if Pcount(4) ~= Pcount_temp(4)
                Pcount_temp(4) = Pcount(4);
                bonus = -5;
                QR(OsIdx,OaIdx) = (1-learnRate)*QR(OsIdx,OaIdx) + learnRate * ( RR(sIdx) + discount*max(QR(sIdx,:)) - QR(sIdx,aIdx) + bonus);
            end
            
            Compare = [test(1),test(2),dist];
            epsilon = epsilon*epsilonDecay; %Probability
            
            %The if there have been a change in pickup in %, if time has passed
            %a defined value or if all robots are stopped.
            if ((Pcount(1) > Pcount_temp(1) || Pcount(2) > Pcount_temp(2) || Pcount(3) > Pcount_temp(3)) && vrep.simxGetConnectionId(clientID)~=-1) %|| TT == 0
                %reinforcement for robots
                if Pcount(1) > 0 Pcount_temp(1) = Pcount(1)*1.01; end%Saved the pickup for the robot
                if Pcount(2) > 0 Pcount_temp(2) = Pcount(2)*1.01; end%Saved the pickup for the robot
                if Pcount(3) > 0 Pcount_temp(3) = Pcount(3)*1.01; end%Saved the pickup for the robot
                
                % Interpolate the state within our discretization (ONLY for choosing
                % the action. We do not actually change the state by doing this!)
                [~,sIdx] = min(sum((statesR - repmat(Compare,[size(statesR,1),1])).^2,2));
                % Choose an action:
                % EITHER 1) pick the best action according the Q matrix (EXPLOITATION). OR
                % 2) Pick a random action (EXPLORATION)
                if (rand()>epsilon || TT == Steps) && rand()<=successRate
                    [~,aIdx] = max(QR(sIdx,:)); %Best action.
                else
                    aIdx = randi(length(actionsR),1); %Randome action.
                end
                
                if aIdx <= 0 || aIdx >= 5
                    aIdx = aIdx_temp;
                else
                    aIdx_temp = aIdx;
                end
                
                if aIdx == 1
                    Pcount_Stop(1) = 0;%Stop or start the robot
                    Pcount_Stop(2) = 0;%Stop or start the robot
                elseif aIdx == 2
                    Pcount_Stop(1) = 0;%Stop or start the robot
                    Pcount_Stop(2) = 1;%Stop or start the robot
                elseif aIdx == 3
                    Pcount_Stop(1) = 1;%Stop or start the robot
                    Pcount_Stop(2) = 0;%Stop or start the robot
                elseif aIdx == 4
                    Pcount_Stop(1) = 1;%Stop or start the robot
                    Pcount_Stop(2) = 1;%Stop or start the robot
                end
                
                if ( 1.31 <= test(1) && test(1) <= 1.35)
                    if ( 1.31 <= test(2) && test(2) <= 1.35)
                        if ( 1.31 <= test(3) && test(3) <= 1.35)
                            if mod(Start,1) == 0
                                disp(test)
                                fprintf("...........................................\n");
                                fprintf("YEAH!!!!!  \n");
                                disp(Start);
                                fprintf("...........................................\n");
                            end
                            Start = Start+1;
                            bonus = bonus+90;
                            if  mod(Start,25) == 0 && DoSave == 0
                                %For Ecxel
                                filename = 'testdata3.xlsx';
                                Save = QR;
                                xlRange = num2str(1);
                                xlswrite(filename,Save,sheet,xlRange)
                            end
                        end
                    end
                end
                
                epsilon = epsilon*epsilonDecay; %Probability
                
                if mod(TT,1) == 0
                    write(Pcount, Pcount_Stop, statesR, sIdx);
                end
                
                %Check if any robot is above the goal, then give them minus.
                if ((test(1) >= goalPt(1)+0.05) || (test(2) >= goalPt(2)+0.05) || (test(3) >= goalPt(3)+0.05))
                    bonus = bonus-15*DGR(sIdx);
                elseif ((test(1) <= goalPt(1)-0.05) || (test(2) <= goalPt(2)-0.05) || (test(3) <= goalPt(3)-0.05))
                    bonus = bonus-15*DGR(sIdx);
                else
                end
                
                if lostBox ~= lostBox_temp
                    lostBox_temp = lostBox;
                    bonus = bonus-100;
                else
                end
                
                %Send it to v-rep
                ComTestVPack = vrep.simxPackFloats(Pcount_Stop);
                [rtn] = vrep.simxSetStringSignal(clientID,'StopData',ComTestVPack,vrep.simx_opmode_oneshot);
                
                %Update Q-learning.
                QR(OsIdx,OaIdx) = (1-learnRate)*QR(OsIdx,OaIdx) + learnRate * ( RR(sIdx) + discount*max(QR(sIdx,:)) - QR(sIdx,aIdx) + bonus);
                OsIdx = sIdx;
                OaIdx = aIdx;
                
                if Flag == 0
                    Pcount_tempDGR = PcountDGR;
                    Flag = 1;
                end
                Pcount_TT = PcountDGR-Pcount_tempDGR;
                TTDGR = (Pcount_TT(1) + Pcount_TT(2) + Pcount_TT(3)); %Total amount of blobs picked up.
                testDGR = [round((Pcount_TT(1)/TTDGR)+1,2),round((Pcount_TT(2)/TTDGR)+1,2),round((Pcount_TT(3)/TTDGR)+1,2)];
                DGR2 = distanceGoal(testDGR(1),testDGR(2)); %Distance to the goal from each state.
                
                %For Ecxel
                filename = 'testdata.xlsx';
                Save = DGR2;
                Counter = Counter+1;
                xlRange = num2str(Counter);
                xlswrite(filename,Save,sheet,xlRange)
                
                %                 if success == true || success
                %                     continue;
                %                 end
            end
        end
    end
    toc%Timer end
    if DoSave == 0
        %For Ecxel
        filename = 'testdata3.xlsx';
        Save = QR;
        xlRange = num2str(1);
        xlswrite(filename,Save,sheet,xlRange)
    end
end
disp('Failed connecting to remote API server');
vrep.delete(); % call the destructor!
disp('Program ended');
end

%Print func.
function write(Pcount, Pcount_Stop, statesR, sIdx)
fprintf("-------------------------------------------\n");
fprintf("Pcount")
disp(Pcount)%Pickup count
fprintf("Stop")
disp(Pcount_Stop)%Stop the robots.
fprintf("State")
disp(statesR(sIdx,:))
end