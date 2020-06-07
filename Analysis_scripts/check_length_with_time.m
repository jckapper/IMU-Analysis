%Checks length of vectors and returns the shortest, also returning the
%time vector that the shorter vector occured over, 
function [length_use, t_vect] = check_length_with_time(vector1, vector2, t_1, t_2)
length1 = length(vector1);
length2 = length(vector2);

if length1 > length2
    length_use = length2;
    t_vect = t_2;
    
else
    length_use = length1;
    t_vect = t_1;
    
end
end
