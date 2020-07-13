%Andrew specification noise function, returns STD of noise for thrust on
%and thrust off time intervals, along with a vector of bias terms 
function [thrust_on_std,thrust_off_std,bias]  = noise_calc(vector_imu, vector_tru, t_tru, t_imu, t_com, F_com)

%Finds shortest vector length and uses that 
[lengthN_use, time_vect] = check_length_with_time(vector_imu, vector_tru, t_imu, t_tru);
errorN = zeros(1,lengthN_use);

%Calculates the error between each point of the two vectors
for i=1:lengthN_use
    difference = vector_imu(i)-vector_tru(i);
    errorN(1,i) = difference;
end

%Initializing variables for the bias calculation
error_minus_bias = zeros(1,length(errorN));
sum = 0;
interval_start = 1;
bias = zeros(1,length(errorN));

%Calculates zero mean error
for i=1:length(errorN)
    %keeps track of the sum and index for the specified interval
    sum = sum + errorN(i);
    
    %Local mean is calculated every two data points because the delta t in 
    %recorded data is very small, that mean is then added to the bias vector
    if mod(i,2) == 0
        disp('yes')
        mean_dt = sum/2;
        bias(1,i) = mean_dt;
            
        %Subtracts the bias from the error data over the specified
        %index interval
        for j=interval_start:i
            error_minus_bias(j) = (errorN(j) - mean_dt);
        end
        
        %Resets counters for the next iteration
        interval_start = i;
        sum = 0;
    end
end

%Combines thrust and the zero mean error with their respective time
%intervals into a two rowed matrix
zero_mean_errorN = error_minus_bias;
zero_mean_with_time = [zero_mean_errorN; time_vect]; %Matrix of zero mean with respective time stamps
thrust_with_time = [F_com; t_com];

%These two vectors will contain the std's for respective time periods, if
%there are three thrust on periods, thrust_on_std will have three entries
thrust_on_std = [];
thrust_off_std = [];

%Used to keep track of current index for iteration
index = 1;

%This while statement only stops after the whole vector has been iterated
%over, this is kept track of using the finished boolean
while true
    %This ends the loop when the last index has been reached.
    if index >= length(thrust_with_time)
        break
    end
    
    data_for_std= [];% Stores data for std calculation
    
    %Checks if the thrust is zero and if so continues to iterate down the
    %thrust vector until thrust is not zero
    if thrust_with_time(1, index) == 0  
        time_off = [];
        %Each index where thrust is zero, the time value is added to the
        %time_off matrix and the index_counter is updated
        while thrust_with_time(1, index) == 0
            
            %Not preallocating because final length is unknown
            time_off = [time_off;thrust_with_time(2,index)];
            index = index + 1;
            
            %Catch statement to check if index_counter has exceeded the
            %length of the thrust matrix
            if index >= length(thrust_with_time)
                break
            end
        end
        
        %Iterates over the zero mean error data, finding all the data
        %points that are within the calculated time interval, storing them
        %in data_for_std and then computes the std of that error data
        for s=1:length(zero_mean_with_time)
            
            %Not preallocating because length of the off vector is unknown
            if zero_mean_with_time(2,s) >= time_off(1) && zero_mean_with_time(2,s) <= time_off(length(time_off))
                data_for_std = [data_for_std;zero_mean_with_time(1,s)];
                
            end
        end
        
        %Adds the calculated std value for the current time interval to the
        %storage vector, if not enough data points to approximate gaussian
        %uses a negative placeholder, same preallocation reasoning
        if length(data_for_std) < 50
            thrust_off_std = [thrust_off_std;-10];
        else
            thrust_off_std = [thrust_off_std;STD(data_for_std)];
        end
            
    else
        time_on = [];
        %If thrust is on, this code block triggers, performs the exact same
        %function as above but for non-zero thrust values
        while thrust_with_time(1, index) ~= 0
            
            %Each index where thrust is non-zero, the time value is added to the
            %time_on matrix and the index_counter is updated, Not 
            %preallocating because length of the off vector is unknown
            time_on = [time_on;thrust_with_time(2,index)];
            index = index + 1;
            
            %Catch statement to check if index_counter has exceeded the
            %length of the thrust matrix
            if index >= length(thrust_with_time)
                break
            end     
        end
        
        %Iterates over the zero mean error data, finding all the data
        %points that are within the calculated time interval, storing them
        %in data_for_std and then computes the std of that error data
        for s=1:length(zero_mean_with_time)
            
            %Not preallocating because length of the off vector is unknown
            if zero_mean_with_time(2,s) >= time_on(1) && zero_mean_with_time(2,s) <= time_on(length(time_on))
                data_for_std = [data_for_std;zero_mean_with_time(1,s)];
                
            end
        end
        
        %Adds the calculated std value for the current time interval to the
        %storage vector, if not enough data points to approximate gaussian
        %uses a negative placeholder (-10), same preallocation reasoning
        if length(data_for_std) < 50
            thrust_on_std = [thrust_on_std;-10];
        else
            thrust_on_std = [thrust_on_std;STD(data_for_std)];
        end
        
        %-10 is displayed if not enough data points are available in a
        %specified time interval to approximate gaussian (<50 data points)
        
    end
end
% %Plot shows data is converted to zero-mean
% figure
% plot(time_vect, error_minus_bias)
% hold on
% plot(time_vect, errorN)
% xlabel('Time (Seconds)')
% title('Comparing Zero-Mean Error to Original')
% legend('Error Minus Bias','Original Error Data')
% mean_error_minus_bias = mean(error_minus_bias); 
% mean_error = mean(errorN);
% txt = ['Mean Error Minus Bias = ' num2str(mean_error_minus_bias)];
% text(20,1,txt)
% txt = ['Mean Original Error = ' num2str(mean_error)];
% text(20,0.8,txt)
% hold off
end