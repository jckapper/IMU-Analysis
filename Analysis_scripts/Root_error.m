%This calculates RMSE of two vectors with the two vectors as input
function RMSE = Root_error(vector_imu, vector_tru)
MSE_sum = 0;

%Checks length of vectors and uses shortest to iterate
length_use = check_length(vector_imu, vector_tru);

%Calculates and sums the mean square error for all data points
for c=1:length_use
    error = vector_imu(c)-vector_tru(c);
    MSE_sum = MSE_sum + error^2;
end

%Divides by sample size and takes sqrt to find RMSE
RMSE = sqrt(MSE_sum/length_use);
end