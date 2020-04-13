classdef open_loop_LLC < low_level_controller
% 14 Mar 2020

    properties

    end
    
    methods
        %% constructor
        function LLC = open_loop_LLC(varargin)
            n_agent_states = 4 ;
            n_agent_inputs = 2 ;
            
            LLC = parse_args(LLC,'n_agent_states',n_agent_states,...
                'n_agent_inputs',n_agent_inputs,varargin{:}) ;
        end
        
        %% get control inputs
        function U = get_control_inputs(~,~,~,~,~,U_des,~)
    
            % create output
            U = U_des ;
        end
    end
end