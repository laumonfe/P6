%   This code is inspired by Matthew Sheen, 2015 and is avalable here:
%   "https://se.mathworks.com/matlabcentral/fileexchange/57882-reinforcement-learning-example-pendulum-controller-w--animation"
%   The code have been made to solve your project but some code is still
%   there from the original code, credits go the Matthew Sheen for these parts of the code.

function C_Robot_RL_With_Count_V7_Two_Robots()
disp('Program started');
%For the connection to V-rep
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%For Excel to save the data
Counter = 0;
sheet = 1;

%To save the information from V-rep and to V-rep
Pcount(1:4) = 1;%Pick up counter
Pcount(4) = 0;%Pick up counter
Pcount_temp(1:4) = 0;%Pick up counter
Pcount_Stop(1:4) = 0;%Stop counter
Pcount_Stop(4) = 1;%Stop counter

%For calculating stuff for the reinforcement
goalPt = [(1/3)+1,0,(1/3)+1]; %The goal point for the agent.
distanceGoal = @(x)sqrt((x-goalPt(1)).^2+((3-(x))-goalPt(3)).^2); %The distance to the goal from where we are.
rewardFuncR = @(x)2.^-(5*x-6);%Reward func
TT = (Pcount(1) + Pcount(3)); %Total amount of blobs picked up.
test = [round((Pcount(1)/TT)+1,2),round((Pcount(3)/TT)+1,2)];%Picked up in procent %

%Values for the reinforcement learning.
learnRate = 0.99;
epsilon = 0.50;
epsilonDecay = 0.98;
discount = 0.9;%discount factor
successRate = 1;
actionsR = [0,1];%Action for the  robot

%All the different combination of action for all robots
Rob1 = 1:0.01:2;
Rob3 = -1.5:0.1:0;

%Clobal values for the program
Start = 1;
OsIdx = 1;
OaIdx = 1;
lostBox = 0;
lostBox_temp = 0;
p_temp = 0;
aIdx_temp = 0;
minus1 = 0;

%--------------------
DoSave = 0;
RunPhase1 = 0;
Phase1 = 20000;
Phase2 = 50;
Steps = 100;
%--------------------

%Create an array of all the combination of actions for the robots
statesR=zeros(length(Rob1)*length(Rob3),2);
indexR=1;
for j = 1:length(Rob1)
    for i = 1:length(Rob3)
        statesR(indexR,1)=Rob1(j);
        statesR(indexR,2)=Rob3(i);
        indexR=indexR+1;
    end
end
DGR = distanceGoal(statesR(:,1)); %Distance to the goal from each state.
RR = rewardFuncR(DGR(:,:));%Reward from each state.
if DoSave == 0 || DoSave == 1
    QR = xlsread('testdata4.xlsx');
    if QR == 0
        QR = repmat(RR,[1,2]);
    end
else
    QR = repmat(RR,[1,2]); %Reward from each action.
end
%-----------------------Init-------------------------
%For fast learning so we dont have to wait for the sim.
tic %Timer start
while (Start <= Phase1 && RunPhase1 == 0 && vrep.simxGetConnectionId(clientID)~=-1 && clientID>-1)
    %success = false;
    Pcount(1:3) = 1;
    Pcount_Stop(1:2) = 0;
    TT = 0;
    bonus = 0;
    %     fprintf("-------------------------------------------\n");
    for TT = 1:Steps
        if ~(vrep.simxGetConnectionId(clientID)~=-1) && ~(clientID>-1)
            break
        end
        
        TT = (Pcount(1) + Pcount(3)); %Total amount of blobs picked up.
        test = [round((Pcount(1)/TT)+1,2),round((Pcount(3)/TT)+1,2)];%Picked up in procent %
        [~,sIdx] = min(sum((statesR - repmat(test,[size(statesR,1),1])).^2,2));%Find the what state is closed to the state we are in.
        if (rand()>epsilon || TT == Steps) && rand()<=successRate%Greedy policy
            [~,aIdx] = max(QR(sIdx,:)); %Best action.
        else%non-Greedy policy
            aIdx = randi(length(actionsR),1); %Randome action.
        end
        %Do the  action
        if aIdx == 1
            Pcount_Stop(1) = 0;%Stop or start the robot
            %Pcount_Stop(2) = 0;%Stop or start the robot
        elseif aIdx == 2
            Pcount_Stop(1) = 1;%Stop or start the robot
            %Pcount_Stop(2) = 0;%Stop or start the robot
        end
        
        if Pcount_Stop(1) == 0
            Pcount(1) = (Pcount(1) + 1);%Stop or start the robot
            RobotWork = 1;
        else
            RobotWork = 0;
        end
        
        if rand()>=(0+0.50*RobotWork)
            Pcount(3) = (Pcount(3) + 1);%Stop or start the robot
        end
        
        if ( 1.47 <= test(1) && test(1) <= 1.53)
            if ( 1.47 <= test(2) && test(2) <= 1.53)
                if TT > 3
                    Start = Start+1;
                    if mod(Start,50) == 0
                        disp(test)
                        fprintf("-------------------------------------------\n");
                        fprintf("YEAH!!!!!  \n");
                        disp(Start);
                        fprintf("-------------------------------------------\n");
                    end
                    %success = true;
                    bonus = 75;
                    if  mod(Start,250) == 0 && DoSave == 0
                        %For Ecxel
                        filename = 'testdata4.xlsx';
                        Save = QR;
                        xlRange = num2str(1);
                        xlswrite(filename,Save,sheet,xlRange)
                    end
                end
            end
        end
        
        epsilon = epsilon*epsilonDecay; %Probability
        
        %         if mod(episode,100) == 0 && success == false
        %             disp(test)
        %         end
        
        if ((test(1) >= goalPt(1)+0.05) || (test(2) >= goalPt(3)+0.05))
            bonus = bonus-15*DGR(sIdx);
        elseif ((test(1) <= goalPt(1)-0.05) || (test(2) <= goalPt(3)-0.05))
            bonus = bonus-15*DGR(sIdx);
        else
        end
        
        for p_temp = 0:15
            Compare = [test(1),p_temp];
            [~,snewIdx] = min(sum((statesR - repmat(Compare,[size(statesR,1),1])).^2,2));
            QR(sIdx,aIdx) = (1-learnRate)*QR(sIdx,aIdx) + learnRate * (RR(snewIdx) + discount*max(QR(snewIdx,:)) - QR(sIdx,aIdx) + bonus );
        end
        
        %         if success == true || success
        %             break;
        %         end
    end
end
toc%Timer end
if DoSave == 0
    %For Ecxel
    filename = 'testdata4.xlsx';
    Save = QR;
    xlRange = num2str(1);
    xlswrite(filename,Save,sheet,xlRange)
end
%Reset Eps
epsilon = 0.01;
%-----------------------V-rep-------------------------
while (clientID>-1)
    Start = 1;
    while (Start <= Phase2 && vrep.simxGetConnectionId(clientID)~=-1)
        Want1 = 0;%Reset
        Pcount_temp(1:4) = 0;
        Pcount_Stop(1:2) = 0;
        %Connection to the v-rep for the pickup count.
        while Want1 == 0 && clientID>-1
            [err,signal]=vrep.simxReadStringStream(clientID,'PcountABC',vrep.simx_opmode_streaming);
            [err,signal]=vrep.simxReadStringStream(clientID,'PcountABC',vrep.simx_opmode_buffer);
            if (err==vrep.simx_return_ok)
                Pcount_temp2(1:4) = vrep.simxUnpackFloats(signal);
                Pcount(1:4) = (vrep.simxUnpackFloats(signal)-Pcount_temp2);
                TT = (Pcount(1) + Pcount(3)); %Total amount of blobs picked up.
                test = [round((Pcount(1)/TT)+1,2),round((Pcount(3)/TT)+1,2)];
                Want1 = 1;
            end
        end
        tic %Timer start
        %success = false;
        bonus = 0;
        episode = 0;
        while TT <= Steps
            if vrep.simxGetConnectionId(clientID)==-1
                break;
            end
            Want1 = 0;%Reset
            Want2 = 0;%Reset
            %Connection to the v-rep for the pickup count.
            while Want1 == 0 && clientID>-1
                [err,signal]=vrep.simxReadStringStream(clientID,'PcountABC',vrep.simx_opmode_streaming);
                [err,signal]=vrep.simxReadStringStream(clientID,'PcountABC',vrep.simx_opmode_buffer);
                if (err==vrep.simx_return_ok)
                    Pcount(1:4) = (vrep.simxUnpackFloats(signal)-Pcount_temp2);
                    TT = (Pcount(1) + Pcount(3)); %Total amount of blobs picked up.
                    test = [round((Pcount(1)/TT)+1,2),round((Pcount(3)/TT)+1,2)];
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
                    Pakke(51);
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
                [~,sIdx] = min(sum((statesR - repmat(Compare,[size(statesR,1),1])).^2,2));
                QR(OsIdx,OaIdx) = (1-learnRate)*QR(OsIdx,OaIdx) + learnRate * ( RR(sIdx) + discount*max(QR(sIdx,:)) - QR(sIdx,aIdx) + bonus);
            end
            
            Compare = [test(1),dist];
            epsilon = epsilon*epsilonDecay; %Probability
            
            %The if there have been a change in pickup in %, if time has passed
            %a defined value or if all robots are stopped.
            if ((Pcount(1) > Pcount_temp(1) || Pcount(3) > Pcount_temp(3)) && vrep.simxGetConnectionId(clientID)~=-1) %|| TT == 0
                %reinforcement for robots
                if Pcount(1) > 0 Pcount_temp(1) = Pcount(1)*1.01; end%Saved the pickup for the robot
                if Pcount(3) > 0 Pcount_temp(3) = Pcount(3)*1.01; end%Saved the pickup for the robot
                
                % Interpolate the state within our discretization (ONLY for choosing
                % the action. We do not actually change the state by doing this!)
                [~,sIdx] = min(sum((statesR - repmat(Compare,[size(statesR,1),1])).^2,2));
                % Choose an action:
                % EITHER 1) pick the best action according the Q matrix (EXPLOITATION). OR
                % 2) Pick a random action (EXPLORATION)
                if (rand()>epsilon || episode == Steps) && rand()<=successRate
                    [~,aIdx] = max(QR(sIdx,:)); %Best action.
                else
                    aIdx = randi(length(actionsR),1); %Randome action.
                end
                
                if aIdx <= 0 || aIdx >= 5
                    aIdx = aIdx_temp;
                else
                    aIdx_temp = aIdx;
                end
                
                %Do the  action
                if aIdx == 1
                    Pcount_Stop(1) = 0;%Stop or start the robot
                    %Pcount_Stop(2) = 0;%Stop or start the robot
                elseif aIdx == 2
                    Pcount_Stop(1) = 1;%Stop or start the robot
                    %Pcount_Stop(2) = 0;%Stop or start the robot
                end
                
                if ( 1.47 <= test(1) && test(1) <= 1.53)
                    if ( 1.47 <= test(2) && test(2) <= 1.53)
                        disp(test)
                        fprintf("...........................................\n");
                        fprintf("YEAH!!!!!  \n");
                        disp(Start);
                        fprintf("...........................................\n");
                        %success = true;
                        Start = Start+1;
                        bonus = bonus+75;
                        if  mod(Start,25) == 0 && DoSave == 0
                            %For Ecxel
                            filename = 'testdata4.xlsx';
                            Save = QR;
                            xlRange = num2str(1);
                            xlswrite(filename,Save,sheet,xlRange)
                        end
                    end
                end
                
                epsilon = epsilon*epsilonDecay; %Probability
                
                episode = episode+1;
                if mod(episode,1) == 0
                    write(Pcount, Pcount_Stop, statesR, sIdx);
                end
                
                %Check if any robot is above the goal, then give them minus.
                if ((test(1) >= goalPt(1)+0.05) || (test(2) >= goalPt(3)+0.05))
                    bonus = bonus-15*DGR(sIdx);
                elseif ((test(1) <= goalPt(1)-0.05) || (test(2) <= goalPt(3)-0.05))
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
                
                %                 if success == true || success
                %                     continue;
                %                 end
            end
        end
    end
    toc%Timer end
    if DoSave == 0
        %For Ecxel
        filename = 'testdata4.xlsx';
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