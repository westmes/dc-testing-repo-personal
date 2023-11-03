classdef HWPortType < Simulink.IntEnumType
    % HWPortType Enumeration type definition for use with System Composer profile

    enumeration
        Unset(0)
        CAN(1)
        LIN(2)
        FlexRay(3)
        Ethernet(4)
    end

end
