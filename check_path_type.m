function [path_type,counter]=check_path_type(turn_point,turn_point_buffer)
path_type=1;
counter=1;
for o=1:size(turn_point_buffer,1)
               
                if(turn_point_buffer(o,:)==turn_point)
                    path_type=2;
                    counter=counter+1;
                end
end
