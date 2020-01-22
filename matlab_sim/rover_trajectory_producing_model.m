function zd = rover_trajectory_producing_model(~,z,u,scale,distance_scale)

    wb = .32;
    zd = zeros(2,1);
      
        w = distance_scale*u(1)*u(2)/wb;
        zd(1) = u(1)-(w*z(2));
        zd(2) = (w*z(1));
        
        zd = zd*scale;
                            
end