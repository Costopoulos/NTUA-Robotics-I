%% Robotics I 2020 - 2021
%% Kostopoulos Konstantinos
%% AM : 03117043
%% Part B : Simulation 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc;

l = zeros(1,6);%array indices must be positive values
                %we've got l0 - l5 so l1 - l6
%all are in cm
l(3) = 10.0; %this is l2
l(5) = 7.0;  %this is l4
l(6) = 5.0;

dt = 0.05;    
Tf = 10.0; 	% 10sec duration of motion 

t=0:dt:Tf; %from 0 to 10 with an interval of 0.02 units -> thus 10/0.02 + 1 = 501 points
             %the +1 is because matlab counts from 1 

%% Question 6 - Task-space Trajectory 
             
%xd0,td0,yd1: intkial/final end-point postkion --> desired task-space trajectory  


for i=0:1 %how many times will the oscillation take place. I chose two (back&forth)
    if (mod(i,2)==0)
        p_A=[6.00;7.00;20.00]; %pA is always (6,7,20), just writing it this way to avoid writing code
        p_B=[1.00;1.00;20.00];
    else                     %when the oscillation is from pB to pA
        p_B=[6.00;7.00;20.00];
        p_A=[1.00;1.00;20.00];
    end


    a1 = 1; %these 3 are obviously equal to 1
    b1 = 1;
    c1 = 1;

    a2 = (3/Tf^2)*(p_B(1) - p_A(1));
    a3 = -(2/Tf^3)*(p_B(1) - p_A(1));

    b2 = (3/Tf^2)*(p_B(2) - p_A(2));
    b3 = -(2/Tf^3)*(p_B(2) - p_A(2));

    c2 = (3/Tf^2)*(p_B(3) - p_A(3)); %obviously = 0
    c3 = -(2/Tf^3)*(p_B(3) - p_A(3)); %obviously = 0

    %disp(sprintf('%d is ', c2)); %c2=c3=0
    % Example of desired trajectory : linear segment (x0,y0)-->(x1,y1); Time duration: Tf; 
    disp('Initialising Desired Task-Space Trajectory (Motion Profile) ...');  
    disp(' ');

    xd = zeros(length(t)); %no need for these, they just improve MATLAB code performance
    yd = zeros(length(t)); %this is because, if not initialized, they get their values by
    %zd = zeros(length(t)); %copying the values of a1*p_A(1) + a2*t(k)^2 + a3*t(k)^3 everytime
                           %whereas, if defined, that only happens 1 time.
    
    xd(1) = p_A(1); 
    yd(1) = p_A(2); 
    zd(1) = p_A(3); 
    for k=2:length(t)    
       xd(k) = p_A(1) + a2*t(k)^2 + a3*t(k)^3;    
       yd(k) = p_A(2) + b2*t(k)^2 + b3*t(k)^3;
       zd(k) = p_A(3) + c2*t(k)^2 + c3*t(k)^3;
    end  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% Question 7 - Kinematic simulation
    disp(sprintf('Kinematic Simulation number %d\n', i)); 
    
    for m=1:length(t)
        pe_x=xd(m);
        pe_y=yd(m);
        pe_z=zd(m);
        
        qd3(m) = acos((pe_x^2 + pe_y^2 + pe_z^2 - l(3)^2 - l(5)^2 - l(6)^2)/(2 * l(5) * l(6)));%l3 = l2, l5=l4,...
        qd2(m) = acos((sin(qd3(m)) * l(6) * pe_y + (l(5) + l(6) * cos(qd3(m))) * sqrt(l(5)^2 + l(6)^2 + 2 * cos(qd3(m)) * l(5) * l(6) - pe_y^2))/(l(5)^2 + l(6)^2 + 2 * cos(qd3(m)) * l(5) * l(6)));
        c23 = cos(qd2(m) + qd3(m));
        qd1(m) = acos(((l(5) * cos(qd2(m)) + l(6) * c23) * (-1) * pe_z + l(3) * sqrt((l(5) * cos(qd2(m)) + l(6) * c23)^2 + l(3)^2 - pe_z^2))/((l(5) * cos(qd2(m)) + l(6) * c23)^2 + l(3)^2));
    end
            
    for it=1:length(t)
        xd1(it) = 0;  
        yd1(it) = 0; 
        zd1(it) = 0;

        xd2(it) = cos(qd2(it)) * sin(qd1(it)) * l(5); %again reminding -> l5=l4, l6=l5 cause Matlab counts from 1
        yd2(it) = l(5) * sin(qd2(it));
        zd2(it) = -cos(qd1(it)) * cos(qd2(it)) * l(5);
        
        xde(it) = cos(qd1(it)) * l(3) + sin(qd1(it)) * cos(qd2(it) + qd3(it)) * l(6) + sin(qd1(it)) * cos(qd2(it)) * l(5);
        yde(it) = sin(qd2(it) + qd3(it)) * l(6) + sin(qd2(it)) * l(5);
        zde(it) = sin(qd1(it)) * l(3) - cos(qd1(it)) * cos(qd2(it)) * l(5) - cos(qd1(it)) * cos(qd2(it) + qd3(it)) * l(6);
    end

%% Plotting   

    fig1 = figure; 
    subplot(2,3,1); 
    plot(t,xde(:)); 
    ylabel('Pe,x (cm)'); 
    xlabel('Time (sec)');  

    subplot(2,3,2); 
    plot(t,yde(:)); 
    ylabel('Pe,y (cm)'); 
    xlabel('Time (sec)');
    title('Desired Position of the End-Effector');

    subplot(2,3,3); 
    plot(t,zd(:)); 
    ylabel('Pe,z (cm)'); 
    xlabel('Time (sec)');
    
    Pe=[xde(:) yde(:) zde(:)];
    fig2 = figure;  
    Ve = zeros(length(t),3);
    for q=2:length(t)
        Ve(q,:) = Pe(q,:) - Pe(q-1,:);
    end

    subplot(2,3,1); 
    plot(t,Ve(:,1)*(length(t)-1)*180/(Tf*pi)); 
    ylabel('Vx (cm/sec)'); 
    xlabel('Time (sec)');  

    subplot(2,3,2); 
    plot(t,Ve(:,2)*(length(t)-1)*180/(Tf*pi));  
    ylabel('Vy (cm/sec)'); 
    xlabel('Time (sec)');   
    title('Linear Velocity of the End-Effector');

    subplot(2,3,3); 
    plot(t,Ve(:,3)*(length(t)-1)*180/(Tf*pi)); 
    ylabel('Vz (cm/sec)'); 
    xlabel('Time (sec)');  
    
    fig3=figure;
    subplot(2,3,1); 
    plot(t,qd1(:)*180/pi); 
    ylabel('q1 (degrees)'); 
    xlabel('Time (sec)');  

    subplot(2,3,2); 
    plot(t,qd2(:)*180/pi); 
    ylabel('q2 (degrees)'); 
    xlabel('Time (sec)');   
    title('Angles of the Joints');

    subplot(2,3,3); 
    plot(t,qd3(:)*180/pi); 
    ylabel('q3 (degrees)'); 
    xlabel('Time (sec)'); 


    qd=[qd1(:) qd2(:) qd3(:)];
    fig4 = figure;  
    subplot(1,3,1);
    u = zeros(length(t),3);
    for q=2:length(t)
        u(q,:) = qd(q,:) - qd(q-1,:);
    end
    
    subplot(2,3,1); 
    plot(t,u(:,1)*(length(t)-1)*180/(Tf*pi)); 
    ylabel('ù1 (deg/sec)'); 
    xlabel('Time (sec)');  

    subplot(2,3,2); 
    plot(t,u(:,2)*(length(t)-1)*180/(Tf*pi));  
    ylabel('ù2 (deg/sec)'); 
    xlabel('Time (sec)');   
    title('Angular Velocity of the Joints');

    subplot(2,3,3); 
    plot(t,u(:,3)*(length(t)-1)*180/(Tf*pi)); 
    ylabel('ù3 (deg/sec)'); 
    xlabel('Time (sec)');


%% Animation  

    fig2 = figure;  
    axis([-l(3) l(3) -(l(5)+l(6)) (l(5)+l(6)) 0 25]) %setting axis
    axis on 
    hold on 
    xlabel('x (cm)'); 
    ylabel('y (cm)'); 
    zlabel ('z (cm)');
    plot3([0],[0], [0] ,'o');
    if (mod(i,2)==0)
        text(p_A(1), p_A(2), p_A(3), 'A');
        text(p_B(1), p_B(2), p_B(3), 'B');
    else
        text(p_A(1), p_A(2), p_A(3), 'B');
        text(p_B(1), p_B(2), p_B(3), 'A');
    end
    dtk=20; %Tf/dt = 10/0.05 = 200 points, so I choose 10 points = 200/20
    for tk=1:dtk:length(t) 
           pause(1); %pauses for 1 sec to show the movement in frames	   
           plot3([0,xd1],[0,yd1], [0,zd1]);
           plot3([xd1],[yd1],[zd1],'o');  
           plot3([xd1(tk),xd2(tk)],[xd1(tk),yd2(tk)], [zd1(tk),zd2(tk)]);
           plot3([xd2(tk)],[yd2(tk)], [zd2(tk)],'o');  
           plot3([xd2(tk),xde(tk)],[yd2(tk),yde(tk)],[zd2(tk),zde(tk)]);	
           plot3([xde(tk)],[yde(tk)],[zde(tk)],'r*');   
    end

end
