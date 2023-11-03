classdef ImplLanguage < Simulink.IntEnumType
    % ImplLanguage Enumeration type definition for use with System Composer profile

    enumeration
        Unset(0)
        Simulink(1)
        Cpp(2)
        MATLAB(3)
        C(4)
    end

end
