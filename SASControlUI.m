function varargout = SASControlUI(varargin)
% SASCONTROLUI MATLAB code for SASControlUI.fig
%      SASCONTROLUI, by itself, creates a new SASCONTROLUI or raises the existing
%      singleton*.
%
%      H = SASCONTROLUI returns the handle to a new SASCONTROLUI or the handle to
%      the existing singleton*.
%
%      SASCONTROLUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SASCONTROLUI.M with the given input arguments.
%
%      SASCONTROLUI('Property','Value',...) creates a new SASCONTROLUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SASControlUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SASControlUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above textMeasuredVout to modify the response to help SASControlUI

% Last Modified by GUIDE v2.5 04-Apr-2016 20:16:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SASControlUI_OpeningFcn, ...
                   'gui_OutputFcn',  @SASControlUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before SASControlUI is made visible.
function SASControlUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SASControlUI (see VARARGIN)

% Choose default command line output for SASControlUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SASControlUI wait for user response (see UIRESUME)
% uiwait(handles.figureMainUI);

fillPortsDropdown(handles);
set(handles.pushbuttonClose,'Enable','off');
set(handles.pushbuttonOn,'Enable','off');
set(handles.pushbuttonOff,'Enable','off');

title('IV Curve');
xlabel('Current');
ylabel('Voltage');
axis([0 8 0 60]);

delete(instrfind());
clc;
format('shorteng');
set(0,'format', get(0, 'format'));
clear origFormat;

handles.serialHandle = 0;
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = SASControlUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbuttonOpen.
function pushbuttonOpen_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonOpen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

selectedValue = get(handles.popupmenuSerialPorts,'Value');
stringList = get(handles.popupmenuSerialPorts,'String');
selectedString = stringList{selectedValue};

if(strfind(selectedString,'COM'))
    % Open serial port
    handles.serialHandle = serial(selectedString);
    handles.serialHandle.Terminator = 'LF';
    % Reduce timeout to 0.5 second (default is 10 seconds)
    handles.serialHandle.Timeout = 5 ;
    % Open virtual serial port
    fopen(handles.serialHandle);
    % Update button state
    updatePortButtonState(handles,updatePortStatus(handles.serialHandle));
    
    guidata(hObject, handles);
    
    updatePrologixVersion(handles,updatePortStatus(handles.serialHandle));
    
    % Set end of GPIB command to CR/LF only
    fprintf(handles.serialHandle, '++eos 2');

    % Suppress "not enough data read before timeout" warning
    warning('off','MATLAB:serial:fread:unsuccessfulRead');

    % Configure as Controller (++mode 1), instrument address 1, and with
    % read-after-write (++auto 1) enabled
    fprintf(handles.serialHandle, '++mode 1');
    fprintf(handles.serialHandle, '++addr 1');
    fprintf(handles.serialHandle, '++auto 0');
else
    errordlg('Error opening serial port; No valid port selected');
end

% --- Executes on button press in pushbuttonClose.
function pushbuttonClose_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonClose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(updatePortStatus(handles.serialHandle))
    scpiCommand(handles.serialHandle, 'OUTP OFF');
    handles.SAS.Enabled = false;
    guidata(hObject, handles);
    fclose(handles.serialHandle);
else
    errordlg('Error closing serial port; Port not opened');
end

updatePortButtonState(handles,updatePortStatus(handles.serialHandle));
updatePrologixVersion(handles,updatePortStatus(handles.serialHandle));

% --- Executes on selection change in popupmenuSerialPorts.
function popupmenuSerialPorts_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenuSerialPorts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenuSerialPorts contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenuSerialPorts


% --- Executes during object creation, after setting all properties.
function popupmenuSerialPorts_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenuSerialPorts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbuttonScan.
function pushbuttonScan_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonScan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

fillPortsDropdown(handles);

% --- Executes on button press in pushbuttonOn.
function pushbuttonOn_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonOn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(updatePortStatus(handles.serialHandle))
    scpiCommand(handles.serialHandle, 'OUTP ON');
    handles.SAS.Enabled = true;
    guidata(hObject, handles);
    measureSASData(hObject, handles);
else
    errordlg('Error enabling; Port not opened');
end


% --- Executes on button press in pushbuttonOff.
function pushbuttonOff_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonOff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(updatePortStatus(handles.serialHandle))
    scpiCommand(handles.serialHandle, 'OUTP OFF');
    handles.SAS.Enabled = false;
    guidata(hObject, handles);
else
    errordlg('Error disabling; Port not opened');
end


% --- Executes on button press in pushbuttonUpdate.
function pushbuttonUpdate_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonUpdate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if checkSASConfigData(handles)
    handles.SAS.Voc = str2num(get(handles.editSASVoc,'String'));
    handles.SAS.Vmp = str2num(get(handles.editSASVmp,'String'));
    handles.SAS.Isc = str2num(get(handles.editSASIsc,'String'));
    handles.SAS.Imp = str2num(get(handles.editSASImp,'String'));
    guidata(hObject, handles);
else
    errordlg('Error in specified SAS config, value out of range.');
end

calcIVCurve(hObject,handles);


% --- Executes on selection change in popupmenuProfileSelector.
function popupmenuProfileSelector_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenuProfileSelector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenuProfileSelector contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenuProfileSelector


% --- Executes during object creation, after setting all properties.
function popupmenuProfileSelector_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenuProfileSelector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function fillPortsDropdown(handles)
disp('Scanning ports');
serialinfo = instrhwinfo('serial');

if isempty(serialinfo.AvailableSerialPorts)
    set(handles.popupmenuSerialPorts,'String','Not available');
    set(handles.pushbuttonOpen,'Enable','off');
else
    set(handles.popupmenuSerialPorts,'String',serialinfo.AvailableSerialPorts);
    set(handles.pushbuttonOpen,'Enable','on');
end

function [ openState ] = updatePortStatus(sportHandle)
if sportHandle == 0;
    openState = false;
else
    if strcmp(get(sportHandle,'Status'),'closed')
        openState = false;
    elseif strcmp(get(sportHandle,'Status'),'open')
        openState = true;
    else
        openState = false;
    end
end

function updatePortButtonState(handles,comPortState)
if comPortState
    set(handles.pushbuttonOpen,'Enable','off');
    set(handles.pushbuttonClose,'Enable','on');
else
    set(handles.pushbuttonOpen,'Enable','on');
    set(handles.pushbuttonClose,'Enable','off');
end

function updatePrologixVersion(handles, connectedState)
if connectedState
    % Send Prologix Controller query version command
    fprintf(handles.serialHandle, '++ver');
    % Read and display response
    versionString = strtrim(fgets(handles.serialHandle));
    versionTextPointer = strfind(versionString, 'version');
    stringLength = size(versionString);
    set(handles.textPrologixVersion,'String',versionString(versionTextPointer:stringLength(2)));
else
    set(handles.textPrologixVersion,'String','version: -');
end 

function editSASVoc_Callback(hObject, eventdata, handles)
% hObject    handle to editSASVoc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editSASVoc as textMeasuredVout
%        str2double(get(hObject,'String')) returns contents of editSASVoc as a double


% --- Executes during object creation, after setting all properties.
function editSASVoc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editSASVoc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editSASVmp_Callback(hObject, eventdata, handles)
% hObject    handle to editSASVmp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editSASVmp as textMeasuredVout
%        str2double(get(hObject,'String')) returns contents of editSASVmp as a double


% --- Executes during object creation, after setting all properties.
function editSASVmp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editSASVmp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editSASIsc_Callback(hObject, eventdata, handles)
% hObject    handle to editSASIsc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editSASIsc as textMeasuredVout
%        str2double(get(hObject,'String')) returns contents of editSASIsc as a double


% --- Executes during object creation, after setting all properties.
function editSASIsc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editSASIsc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editSASImp_Callback(hObject, eventdata, handles)
% hObject    handle to editSASImp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editSASImp as textMeasuredVout
%        str2double(get(hObject,'String')) returns contents of editSASImp as a double


% --- Executes during object creation, after setting all properties.
function editSASImp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editSASImp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function calcIVCurve(hObject,handles)
Isc = handles.SAS.Isc;
Imp = handles.SAS.Imp;
Voc = handles.SAS.Voc;
Vmp = handles.SAS.Vmp;

% Create array
I = linspace(0,Isc,200);

% Calculate relation
Rs = (Voc-Vmp)/Imp;
a = (Vmp*(1+Rs*Isc/Voc)+Rs*(Imp-Isc))/Voc;
N = log(2-2.^a)/log(Imp/Isc);
Vnominator = ((Voc*log(2-(I/Isc).^N))/log(2))-Rs*(I-Isc);
Vdenominator = (1+Rs*Isc/Voc);
V = Vnominator/Vdenominator;

handles.Curve.I = I;
handles.Curve.V = V;

guidata(hObject, handles);
updateGraph(hObject, handles);

function updateGraph(hObject, handles)
[plotHandle, lineHandle1, lineHandle2] = plotyy(handles.Curve.I,handles.Curve.V,handles.Curve.I,handles.Curve.I.*handles.Curve.V);
title('IV / IP Curve');

set(lineHandle1,'Color','blue');
set(lineHandle2,'Color','red');

set(plotHandle,{'ycolor'},{'blue';'red'});
ylabel(plotHandle(1),'Voltage') % left y-axis
axis(plotHandle(1),[0 handles.SAS.Isc*1.05 0 handles.SAS.Voc*1.05]);
ylabel(plotHandle(2),'Power') % right y-axis
axis(plotHandle(2),[0 handles.SAS.Isc*1.05 0 handles.SAS.Voc.*handles.SAS.Isc*1.2]);

hold on
handles.Curve.blueDot = plot(0,0,'o','MarkerFaceColor','blue');
handles.Curve.redDot = plot(0,0,'o','MarkerFaceColor','red');
guidata(hObject, handles);
hold off

function [ valid ] = checkSASConfigData(handles)
emptiesNaN = isempty(str2num(get(handles.editSASVoc,'String'))) + ...
        isempty(str2num(get(handles.editSASVmp,'String'))) + ...
        isempty(str2num(get(handles.editSASIsc,'String'))) + ...
        isempty(str2num(get(handles.editSASImp,'String')));
    
if emptiesNaN
    valid = false;
else
    Voc = str2num(get(handles.editSASVoc,'String'));
    Vmp = str2num(get(handles.editSASVmp,'String'));
    Isc = str2num(get(handles.editSASIsc,'String'));
    Imp = str2num(get(handles.editSASImp,'String'));
    
    errorVal = checkIfWithinRange(Voc,0,60,true) + ...
    checkIfWithinRange(Vmp,0,Voc,false) + ...
    checkIfWithinRange(Isc,0,8,true) + ...
    checkIfWithinRange(Imp,0,Isc,false);

    if errorVal
        valid = false;
    else
        valid = true;
    end
end

function [ error ] = checkIfWithinRange(input,min,max,equalAllowed)
if equalAllowed
    if (input <= max) && (input >= min)
        error = false;
    else
        error = true;
    end
else
    if (input < max) && (input >= min)
        error = false;
    else
        error = true;
    end    
end
    

% --- Executes on button press in pushbuttonUpload.
function pushbuttonUpload_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonUpload (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if checkSASConfigData(handles)
    handles.SAS.Voc = str2num(get(handles.editSASVoc,'String'));
    handles.SAS.Vmp = str2num(get(handles.editSASVmp,'String'));
    handles.SAS.Isc = str2num(get(handles.editSASIsc,'String'));
    handles.SAS.Imp = str2num(get(handles.editSASImp,'String'));
    guidata(hObject, handles);
    
    if(updatePortStatus(handles.serialHandle))
        calcIVCurve(hObject,handles);
        uploadSASData(handles);
        
        set(handles.pushbuttonOn,'Enable','on');
        set(handles.pushbuttonOff,'Enable','on');
        
        checkForErrors(handles); % If errors -> ON button wil be disabled in this function
    else
        errordlg('Error updating; Port not opened');
    end
else
    errordlg('Error in specified SAS config, value out of range.');
end

function [  ] = scpiCommand( port, data, varargin )
%SCPI_COMMAND Write SCPI command to the GPIB bus
%   Detailed explanation goes here

fprintf(port, '%s\n', sprintf(data, varargin{:}));

function [ return_string ] = scpiQuery( port, data, varargin )
%SCPI_QUERY Write SCPI command to the GPIB bus and get data back
%   Detailed explanation goes here

fprintf(port, '%s\n', sprintf(data, varargin{:}));
fprintf(port, '++read eoi');

% Return the string available on the serial port
return_string = fgetl(port);

function uploadSASData(handles)
% Send id query command to R&S UPL Audio Analyzer and display it
disp('Query device ID:');
fprintf('Device identification: %s\n', scpiQuery(handles.serialHandle, '*IDN?'));

% Reset device
disp('Resetting device.');
scpiCommand(handles.serialHandle, '*RST'); % Reset device
scpiCommand(handles.serialHandle, '*CLS'); % Resets IEC/IEEE-bus status register

% Enable power supply in supply mode
disp('Setting power supply to SAS mode.');
scpiCommand(handles.serialHandle, 'OUTP OFF');
scpiCommand(handles.serialHandle, ['CURR:SAS:ISC ' num2str(handles.SAS.Isc) ';IMP ' num2str(handles.SAS.Imp) ';:VOLT:SAS:VOC ' num2str(handles.SAS.Voc) ';VMP ' num2str(handles.SAS.Vmp)]);
%disp(['CURR:SAS:ISC ' num2str(handles.SAS.Isc) ';IMP ' num2str(handles.SAS.Imp) ';:VOLT:SAS:VOC ' num2str(handles.SAS.Voc) ';VMP ' num2str(handles.SAS.Vmp)]);
scpiCommand(handles.serialHandle, 'CURR:MODE SAS');

function checkForErrors(handles)
firstTest = true;

while(firstTest)
    disp('Reading device errors.');
    errorString = strtrim(scpiQuery(handles.serialHandle, 'SYST:ERR?'));
    commaTextPointer = strfind(errorString, ',');
    stringLength = size(errorString);
    errorDescriptor = errorString((commaTextPointer+1):stringLength(2));
    errorCode = errorString(1:(commaTextPointer-1));
    
    if str2num(errorCode) >= 0
        disp('No errors.');
        firstTest = false;
    else
        set(handles.pushbuttonOn,'Enable','off');
        disp(['Error: ' errorString]);
        errordlg(errorString);
    end
end


function measureSASData(hObject, handles)
set(handles.pushbuttonUpdate,'Enable','off');
set(handles.pushbuttonUpload,'Enable','off');

while handles.SAS.Enabled;
    MeasuredVoltage = str2double(strtok(scpiQuery(handles.serialHandle, 'MEAS:VOLT?')));
    MeasuredCurrent = str2double(strtok(scpiQuery(handles.serialHandle, 'MEAS:CURR?')));
    %disp(['Current: ' num2str(MeasuredCurrent) ' Voltage: ' num2str(MeasuredVoltage)]);
    
    set(handles.textMeasuredVout,'String',sprintf('%0.2f',abs(MeasuredVoltage)));
    set(handles.textMeasuredIout,'String',sprintf('%0.2f',abs(MeasuredCurrent)));
    set(handles.textMeasuredPout,'String',sprintf('%0.2f',abs(MeasuredVoltage*MeasuredCurrent)));
    set(handles.Curve.blueDot, 'Xdata', abs(MeasuredCurrent),'Ydata', abs(MeasuredVoltage));
    set(handles.Curve.redDot, 'Xdata', abs(MeasuredCurrent),'Ydata', abs(MeasuredVoltage).*abs(MeasuredCurrent./handles.SAS.Isc*(1.05/1.2)));
    
    pause(0.01);
    
    if hObject
        handles = guidata(hObject);
    else
        handles.SAS.Enabled = false;
    end 
end

pause(0.2);
MeasuredVoltage = str2double(strtok(scpiQuery(handles.serialHandle, 'MEAS:VOLT?')));
MeasuredCurrent = str2double(strtok(scpiQuery(handles.serialHandle, 'MEAS:CURR?')));
%disp(['Current: ' num2str(MeasuredCurrent) ' Voltage: ' num2str(MeasuredVoltage)]);

set(handles.textMeasuredVout,'String',sprintf('%0.2f',abs(MeasuredVoltage)));
set(handles.textMeasuredIout,'String',sprintf('%0.2f',abs(MeasuredCurrent)));
set(handles.textMeasuredPout,'String',sprintf('%0.2f',abs(MeasuredVoltage*MeasuredCurrent)));
set(handles.Curve.blueDot, 'Xdata', abs(MeasuredCurrent),'Ydata', abs(MeasuredVoltage));
set(handles.Curve.redDot, 'Xdata', abs(MeasuredCurrent),'Ydata', abs(MeasuredVoltage).*abs(MeasuredCurrent));
set(handles.pushbuttonUpdate,'Enable','on');
set(handles.pushbuttonUpload,'Enable','on');


% --- Executes during object deletion, before destroying properties.
function figureMainUI_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figureMainUI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when user attempts to close figureMainUI.
function figureMainUI_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figureMainUI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure

disp('Shutting down');
if(updatePortStatus(handles.serialHandle))
    handles.SAS.Enabled = false;
    guidata(hObject, handles);
    scpiCommand(handles.serialHandle, 'OUTP OFF');
    fclose(handles.serialHandle);
end
disp('Done');

delete(hObject);
