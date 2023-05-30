%function to interpolate thrust force

%input: data_array for interpolation
%       tsr at this time step
%       pitch angle at this time step

%output: thrust force at this time
function inter_result = thrust_interp(tsr_idx,theta,Ft_array)
    array = Ft_array(tsr_idx, :);
    x = 0:1:length(array) - 1;              %pitch angle data
    y = array;                              %thrust force data
    inter_result = interp1(x,y,theta);
end