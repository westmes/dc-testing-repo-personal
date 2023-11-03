classdef ReviewStatus < Simulink.IntEnumType
    % ReviewStatus Enumeration type definition for use with System Composer profile

    enumeration
        NeedsReview(0)
        UnderReview(1)
        Reviewed(2)
    end

end
