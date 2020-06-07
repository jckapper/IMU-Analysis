%Checks length of vectors and uses returns the shortest
function length_use = check_length(vector1, vector2)
length1 = length(vector1);
length2 = length(vector2);

if length1 > length2
    length_use = length2;
    
else
    length_use = length1;
    
end
end