%Handmade std function using formula
function std_data = STD(data_vector)
sum_data = 0;
mean_data = mean(data_vector);

%Sums all (data point - mean)^2
for d=1:length(data_vector)
    shifted = (data_vector(d) - mean_data)^2;
    sum_data = sum_data + shifted;
end

%Divides the summation by N-1 for unbiased estimator and then takes sqrt to
%get std
std_data = sqrt(sum_data/(length(data_vector)-1));
end