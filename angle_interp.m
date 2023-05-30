%function to interpolate pitch angle

%input: data_array for interpolation
%       tsr at this time step
%       cp at this time step

%output: pitch angle at this time
function inter_result = angle_interp(tsr_idx,cp,cp_array)
    array = cp_array(tsr_idx, :);       
    x = array;                              %cp data
    y = 0:1:length(array) - 1;              %pitch angle data
    inter_result = interp1(x,y,[cp]);
end