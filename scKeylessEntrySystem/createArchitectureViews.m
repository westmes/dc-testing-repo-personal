%% Creating Architectural Views Programmatically

% Copyright 2019-2020 The MathWorks, Inc.
%% Overview
% You can create an architecture view programmatically. This script
% constains two examples for creating views programmatically.

%% Open the Model
import systemcomposer.query.*;
zcModel = systemcomposer.loadModel('KeylessEntryArchitecture');

%% Example 1: Hardware Component Review Status
% This example shows how to create a filtered view which selects all of
% the hardware components in the model and groups them using the
% ReviewStatus property.

% Construct the query to select all of the hardware components
hwCompQuery = HasStereotype(IsStereotypeDerivedFrom("AutoProfile.HardwareComponent"))

% Use the query to create the view
zcModel.createView("Hardware Component Review Status", ...
    "Select", hwCompQuery, ... % The query to use for the selection
    "GroupBy", {'AutoProfile.BaseComponent.ReviewStatus'}, ... % The stereotype property to qualify by
    "IncludeReferenceModels", true, ... % Include components in referenced models
    "Color", "purple");

% Open the views gallery
zcModel.openViews;

%% Example 2: FOB Locator System Supplier View
% This example shows how to create a freeform view which manually pulls
% the components from the FOB Locator system and then groups them using
% view components for the suppliers.

fobSupplierView = zcModel.createView("FOB Locator System Supplier Breakdown", ...
    "Color", "lightblue");

% Create a new view component for supplier D and add the FOB Locator
% module to it.
supplierD = fobSupplierView.Root.createSubGroup("Supplier D");
fobLocatorModule = zcModel.lookup("Path", "KeylessEntryArchitecture/FOB Locator System/FOB Locator Module");
supplierD.addElement(fobLocatorModule);

% Create a new view component for supplier A
supplierA = fobSupplierView.Root.createSubGroup("Supplier A");

% Add each of the FOB receivers to cross-view component
FOBLocatorSystem = zcModel.lookup("Path", "KeylessEntryArchitecture/FOB Locator System");
[~, receiverComps] = zcModel.find( ...
    contains(systemcomposer.query.Property("Name"), "Receiver"), ... % Find all the components which contain the name "Receiver"
    FOBLocatorSystem.Architecture);

for i = 1:numel(receiverComps)
    % Add each of the components to supplier A
    supplierA.addElement(receiverComps(i));
end

% Open the views gallery
zcModel.openViews;

%% Close the model
zcModel.close('Force');
