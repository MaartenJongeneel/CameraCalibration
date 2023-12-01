function Mocap = OptImport(OptDatalog)
% This function creates the structs for the OBJECT, ROBOT, ENVIRONMENT 
% folders and the Mocap folder for SENSOR_MEASUREMENT. This information is 
% based of the Optitrack data (.csv file) and the info stored in SETUPINFO.
% SETUPINFO is a path to folder containing data about the objects,
% environment and robots that can be used. The specific data that is used
% from SETUPINFO is based on what is used during the experiment, which is
% defined in the Optitrack data file (.csv file). 
%
% INPUTS:    OptDatalog   : .csv file of a recording (OptiTrack datalog)
%
% OUTPUTS:   Mocap        : Contains raw data from the Motion Caputure
%                           System, settings used to create this data, and
%                           postprocessed data.
%% Load OptiTrack data and write it to a struct
%First we need to get rid of the double-quotation marks in the .csv file
chr = fileread(OptDatalog);
chr = strrep( chr, '"', '' );
fid = fopen( OptDatalog, 'w' ); 
fprintf( fid, '%s', chr );
fclose( fid );

% Import the informative header to detect the required setup
HeadOpts = delimitedTextImportOptions("DataLines", [1, 7],"Delimiter",{','},"LeadingDelimitersRule","keep","ConsecutiveDelimitersRule","split","VariableTypes",'string',"EmptyLineRule","read");
Head = readtable(OptDatalog, HeadOpts);                 %Read the header
Head = table2array(Head);                               %Convert header to table
Head(find(Head(:,1)=="Frame")+1:end,:) = [];            %Delete everything after Frame        
Head(Head == "") = missing;                             %Fill empty fields with <missing>

%% Create the SETTINGS folder
%Obtain the unique rigid body names
names = unique(rmmissing(Head(4,3:end)));
unqRbNames = names(~contains(names,{':','Name','Unlabeled','Active'}));
   
%% Intermediate checks
if ~strcmp(Head(circshift(Head=="Rotation Type",1,2)),"Quaternion")
    error("OptImport: OptiTrack rotation type should be quaternion!");
end
%% Setup the main data-import
% Specify column names and types
varnames = ["Frame", "Time(s)"];
i_RPM = find(Head(:,2)=="ID")+1;  % Index of row indicating data type
i_name = find(Head(:,2)=="Name"); % Index of row indicating data type
i_ax  = length(Head(:,1));        % Index of row indicating data type

for ii = 3:length(rmmissing(Head(i_RPM,:)))+2        
    if strcmp(Head(i_name-1,ii),"Rigid Body Marker")     %if type = rigid body marker
        typ = "_RBM";
    else
        typ = "";
    end    
    if strcmp(Head(i_RPM,ii),"Position")
        varnames(1,ii) = cellstr(strcat(Head(i_name,ii), "_P", Head(i_ax,ii), typ)); %Position
    elseif strcmp(Head(i_RPM,ii),"Rotation")
        varnames(1,ii) = cellstr(strcat(Head(i_name,ii), "_R", Head(i_ax,ii), typ)); %Rotation
    elseif strcmp(Head(i_RPM,ii),"Mean Marker Error")
        varnames(1,ii) = cellstr(strcat(Head(i_name,ii), "_MME", typ));              %Mean Marker Error
    elseif strcmp(Head(i_RPM,ii),"Marker Quality")
        varnames(1,ii) = cellstr(strcat(Head(i_name,ii), "_MQ", typ));               %Marker quality
    else
        varnames(1,ii) = cellstr(strcat(Head(i_name,ii), "???"));
        warning("OptImport: Variable Name not recognized!");
    end    
end

%Set Text Import Options
opts = delimitedTextImportOptions("DataLines",[length(Head(:,1))+1 inf],"Delimiter",",","VariableNames",varnames,"PreserveVariableNames",true,"ExtraColumnsRule","ignore","EmptyLineRule","read");
opts.VariableTypes = repmat("double",size(opts.VariableTypes));

% Import the data 
Mocap.datalog.ds = readtable(OptDatalog, opts);
Mocap.datalog.ds.Properties.VariableNames = strrep(Mocap.datalog.ds.Properties.VariableNames," ","_");

%% Post processing of data
% Rearrange quaternions from XYZW to WXYZ
for ii = 1:length(unqRbNames) 
    Mocap.datalog.ds = movevars(Mocap.datalog.ds,strcat(unqRbNames(ii), "_RW"),'Before',strcat(unqRbNames(ii), "_RX"));
end

%Remove unlabeled markers from the dataset
Mocap.datalog.ds(:,startsWith(Mocap.datalog.ds.Properties.VariableNames,"Unlabeled")) = [];

%Because motive puts y-axis in direction of gravity
H1 = [Rx(90), zeros(3,1); zeros(1,3),1]*[Ry(90), zeros(3,1); zeros(1,3),1];

%Calculation of transformation matrices (object w.r.t. Motive origin)
for ii = 1:ceil(height(Mocap.datalog.ds))
    for io = 1:length(unqRbNames)             % for each body
        % Determine the indices of the rotation and position columns
        i_bd.R{1,io}   = contains(string(Mocap.datalog.ds.Properties.VariableNames)',strcat(unqRbNames(io), "_R"));
        i_bd.P{1,io}   = contains(string(Mocap.datalog.ds.Properties.VariableNames)',strcat(unqRbNames(io), "_P"));
        %Compute the transformation matrix
        try
            H = makehgtform('translate', table2array(Mocap.datalog.ds(ii,i_bd.P{1,io})))* ... %translation
                quat2tform(table2array(Mocap.datalog.ds(ii,i_bd.R{1,io}))); %rotation
        catch
            warning('OptImport: Problem with data storage, likely due to framedrops (missing data). Using H = eye(4) instead.');
            H = eye(4);
        end
        %Save the data to the struct
        Mocap.POSTPROCESSING.(unqRbNames(io)).transforms.ds{ii,1} = H1*H;
        Mocap.POSTPROCESSING.(unqRbNames(io)).attr.note = {char(strcat('folder containing transformation matrices of ',{' '},unqRbNames(io),' expressing its pose in terms of frame M. See MoCap/SETTINGS/frame_definition.'))};
        Mocap.POSTPROCESSING.(unqRbNames(io)).transforms.attr.note = {char(strcat('4x4 transformation matrices of ',{' '},unqRbNames(io),' expressing its pose in terms of frame M. See MoCap/SETTINGS/frame_definition.'))};
    end
end
end