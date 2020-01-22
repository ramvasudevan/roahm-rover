function h = plot_2D_msspoly_contour_rover(p,x,l,varargin)
 
%% parse inputs
    if nargin < 3
        l = 0 ;
        varargin = {} ;
    end
    
    % create default inputs
    FillColor = [] ;
    Offset = [0;0] ;
    Scale = 1 ;
    
    % iterate through varargin to find Offset and Scale
    varargin_new = {} ;
    idx_new = 1 ;
    for idx = 1:2:length(varargin)
        switch varargin{idx}
            case 'FillColor'
                FillColor = varargin{idx+1} ;
            case 'Scale'
                Scale = varargin{idx+1};
            otherwise
                varargin_new{idx_new} = varargin{idx} ;
                varargin_new{idx_new+1} = varargin{idx+1} ;
                idx_new = idx_new + 2 ;
        end
    end

%% set up for plotting
    % set up grid for plotting
    x_vec = linspace(-1,1,100) ;
    [X1,X2] = meshgrid(x_vec,x_vec) ;
    X = [X1(:), X2(:)]' ;
    
    % create msspoly surface
    P = reshape(full(msubs(p,x,X)),100,100) ;
    
    X1 = Scale*(X1) + Offset(1) ;
    X2 = Scale*(X2) + Offset(2) ;
    
%% plot
    if ~isempty(FillColor)
        [~,h] = contourf(X1,X2,P,[l l],'Fill','on',varargin_new{:}) ;
        colormap(FillColor)
    else
        h = contour(X1,X2,P,[l l],varargin_new{:}) ;
    end
    
    if nargout < 1
        clear h
    end
end