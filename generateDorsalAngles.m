function [ beta ] = generateDorsalAngles( m, contacts )
beta0=pi/10;
beta=zeros(m,1);%initial value of the dorsal joint angles
contacts=contacts+0.5;
for i=1:m%for each dorsal joint
    if i<contacts(1)%left end
        if i==contacts(1)-0.5
            %the lifting one
            beta(i)=beta0;
        else
            %be flat/ do nothing
        end
    elseif i>contacts(2)&&i<contacts(3)%dorsal joints in between contacts
        %number of dorsal joints in between contacts
        k=contacts(3)-contacts(2);
        if k<=2%only one dorsal joint: nothing can be done
            
        elseif rem(k,2)==1%odd number of dorsal joints
            %if it is the raising one
            if i==contacts(2)+0.5||i==contacts(3)-0.5%raising joints
                beta(i)=beta0;
            else
                if i==(contacts(2)+contacts(3))/2%middle module
                    beta(i)=-2*beta0;
                end
            end
        else
            %even number of dorsal joints
            %if it is the raising one
            if i==contacts(2)+0.5||i==contacts(3)-0.5%raising joints
                beta(i)=beta0;
            else
                if i==(contacts(2)+contacts(3))/2-0.5||i==(contacts(2)+contacts(3))/2+0.5%middle module
                    beta(i)=-beta0;
                end
            end
        end
    else
        if i==contacts(4)+0.5
            beta(i)=beta0;
        end
    end
end
end

