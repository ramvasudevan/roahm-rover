function zd = rover_tracking_model(~,z,u,scale,distance_scale)
    % extract states

    if nargin == 3
        scale = 1;
    end

   zd = zeros(4,1);
   c = [1.6615e-05,-1.9555e-07,3.6190e-06,4.3820e-07,-0.0811,...
       -1.4736,0.1257,0.0765,-0.0140];
      
        zd(3) = distance_scale*((tan(c(1)*u(2)+c(2))*z(4))/(c(3)+c(4)*z(4)^2));
        v_y = zd(3)*(c(8)+c(9)*z(4)^2) ;
        zd(1) = z(4)*cos(z(3))-v_y*sin(z(3));
        zd(2) = z(4)*sin(z(3))+v_y*cos(z(3));
        zd(4) = c(5)+c(6)*(z(4)-u(1))+c(7)*(z(4)-u(1))^2 ;
        
        zd = zd*scale;
        
      
end