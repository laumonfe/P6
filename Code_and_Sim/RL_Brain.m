% Make sure to have the server side running in V-REP:
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19998)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!


function RL_Brain()
disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19998,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
 % it's -1 because the position will never be -1, but could be 0 
    BlobPos = -1;
    BlobTime = -1;
    BlobPaT = -1;
    sDistInfo = [9999,0,0;9999,0,0;9999,0,0]; %distance, ID of blob, a flag or something, and one for each robot
    
    pickupHeight = 0.6373;
    cv = 0.16; % what is cv, conveyor velocity?
    
    
    while (vrep.simxGetConnectionId(clientID)~=-1)
        for k = 1:3
            [err,signal]=vrep.simxReadStringStream(clientID,'BlobPaTData',vrep.simx_opmode_streaming);
            [err,signal]=vrep.simxReadStringStream(clientID,'BlobPaTData',vrep.simx_opmode_buffer);
            % first one initializes that we want  stream of data, second one saves data
            if (err==vrep.simx_return_ok)
                signal;
                BlobPaT = vrep.simxUnpackFloats(signal);
                BlobPos = BlobPaT(1:40);
                BlobTime = BlobPaT(41:80);
            end

            
            if (BlobPaT(1) ~= -1)
                
                if (k == 1) pt = [-0.98637968301773,0.37562590837479,0.68745446205139]; end
                if (k == 2) pt = [0.21362039446831,0.37562590837479,0.68745446205139]; end
                if (k == 3) pt = [1.4136204719543,0.37562590837479,0.68745446205139]; end
                
                sDist = [9999,0,1]; 
                mDist = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
                
                %Calculating distance to blobs
                for j = 1:40
                    if (BlobPos(j) ~= 0)
                        simTime = vrep.simxGetLastCmdTime(clientID)/1000; % change it to seconds
                        p=[-1.4942+((simTime-BlobTime(j))*cv),BlobPos(j),pickupHeight]; %what is the first number?
                        dp=[p(1)-pt(1),p(2)-pt(2),p(3)-pt(3)]; 
                        mDist(j)=sqrt(dp(1)*dp(1)+dp(2)*dp(2)+dp(3)*dp(3)); %Distance to target position
                    else
                        mDist(j) = 9999; 
                    end
                end
                
                
                %Calculating closest blob 
                %Can we get an explaination? 
                for j = 1:40
                    if (j == 1)
                        sDist(1) = mDist(j);
                        sDist(2) = j;
                    end
                    if (mDist(j) < sDist(1) && j ~= 1)
                        sDist(1) = mDist(j);
                        sDist(2) = j;
                    end
                end
                
                if (sDist(1) == 9999)
                    sDist(2) = 0;
                end
                
                sDistInfo(k,1:3) = sDist;
                sDistInfo;
                
                if (k == 1)% why is k always 1?
                    BlobDataPack = vrep.simxPackFloats(sDistInfo(1,1:3));
                    [rtn] = vrep.simxSetStringSignal(clientID,'sBlobDataA',BlobDataPack,vrep.simx_opmode_oneshot);
                end
                
                if (k == 1)
                    BlobDataPack = vrep.simxPackFloats(sDistInfo(2,1:3));
                    [rtn] = vrep.simxSetStringSignal(clientID,'sBlobDataB',BlobDataPack,vrep.simx_opmode_oneshot);
                end
                
                if (k == 1)
                    BlobDataPack = vrep.simxPackFloats(sDistInfo(3,1:3));
                    [rtn] = vrep.simxSetStringSignal(clientID,'sBlobDataC',BlobDataPack,vrep.simx_opmode_oneshot);
                end
                
            end
        end
    end
    
    
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

disp('Program ended');
end