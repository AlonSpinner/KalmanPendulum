function Sfcn_DrawPendulum_OnlyPhysics(block)
setup(block);
end
function setup(block) %runs at t=0 i/o definitions
block.SetSimViewingDevice(true);

%dialog parameters
block.NumDialogPrms = 8;
block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable',...
    'Tunable','Tunable','Tunable','Tunable','Tunable'}; %can change during simulation
%[ControlTs,XLim,YLim,Length,Gravity Acceleration,Mass]

%register number of ports
block.NumInputPorts = 4;
block.NumOutputPorts = 0;

%setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;

%Register the properties of the input ports

%Enable
block.InputPort(1).Complexity     ='Real';
block.InputPort(1).DataTypeId     =-1;
block.InputPort(1).Dimensions     =1;
block.InputPort(1).SamplingMode   ='Sample';

%State
block.InputPort(2).Complexity     ='Real';
block.InputPort(2).DataTypeId     =-1;
block.InputPort(2).Dimensions     =2;
block.InputPort(2).SamplingMode   ='Sample';

%Force
block.InputPort(3).Complexity     ='Real';
block.InputPort(3).DataTypeId     =-1;
block.InputPort(3).Dimensions     =1;
block.InputPort(3).SamplingMode   ='Sample';

%Time
block.InputPort(4).Complexity     ='Real';
block.InputPort(4).DataTypeId     =-1;
block.InputPort(4).Dimensions     =1;
block.InputPort(4).SamplingMode   ='Sample';

%Register sample time
ControlTs=block.DialogPrm(1).Data;
block.SampleTimes = [ControlTs 0]; %[discrete time, offset]

%specify block simStateCompliace
block.SimStateCompliance = 'HasNoSimState';

%register functions
block.RegBlockMethod('InitializeConditions',    @InitializeConditions);
block.RegBlockMethod('Start',                   @Start);
block.RegBlockMethod('Terminate',               @Terminate);
block.RegBlockMethod('Outputs',                 @Outputs);
block.RegBlockMethod('CheckParameters',         @CheckPrms);
block.RegBlockMethod('ProcessParameters',       @ProcessPrms);
end
function ProcessPrms(block) %runs on every dt (Wasnt checked!)
  block.AutoUpdateRuntimePrms;
end
function InitializeConditions(block) %runs on t=0 and when susbystem is enabled
Enable=block.InputPort(1).Data(1);
if ~Enable, return, end

%check if figute exists and valid. if not - reset it
UserData=get(gcbh,'UserData');
if isempty(UserData) %first time simulation is activated
     SetupFigAndUserData(block);
elseif ~ishghandle(UserData.Figure) %figure was deleted
    SetupFigAndUserData(block);
else %figure exists, just clear it and start a new
    SetupFigAndUserData(block,UserData.Figure); %reset figure
end
end
function Outputs(block) %runs on every dt
UserData=get(gcbh,'UserData');
if ~ishghandle(UserData.Figure)
     UserData=SetupFigAndUserData(block); %set figure to a new start
end

%fix limits if asked in sfunction parameters
XLim=block.DialogPrm(2).Data; xlim(UserData.Axes,XLim);
YLim=block.DialogPrm(3).Data; ylim(UserData.Axes,YLim);

%Draw Measured Data
Theta=block.InputPort(2).Data(1);
Omega=block.InputPort(2).Data(2);
F=block.InputPort(3).Data;
L=block.DialogPrm(4).Data;
g=block.DialogPrm(5).Data;
m=block.DialogPrm(6).Data;
FNorm=block.DialogPrm(7).Data;
OmegaNorm=block.DialogPrm(8).Data;
BulbX=L*cos(-pi/2+Theta);
BulbY=L*sin(-pi/2+Theta);

T=makehgtform('translate',[BulbX,BulbY,0]);
UserData.hBulbTransform.Matrix=T;

UserData.hString.XData(2)=BulbX;
UserData.hString.YData(2)=BulbY;
UserData.hVel.XData=BulbX;
UserData.hVel.YData=BulbY;
UserData.hVel.UData=(Omega/OmegaNorm)*L*cos(Theta);
UserData.hVel.VData=(Omega/OmegaNorm)*L*sin(Theta);
UserData.hForce.XData=BulbX;
UserData.hForce.YData=BulbY;
UserData.hForce.UData=F/FNorm;

%Update time text
Time=block.InputPort(4).Data(1);
UserData.hTime.String=sprintf('Time %g[s]',Time);

drawnow limitrate
end
%% Auxiliary functions
function UserData=SetupFigAndUserData(block,varargin)
if nargin<2 %figure was not provided in input
    %Create figure
    FigureName='OnlyPhysics';
    Fig = figure(...
        'Name',              FigureName,...
        'NumberTitle',        'off',...
        'IntegerHandle',     'off',...
        'Color',             [1,1,1],...
        'MenuBar',           'figure',...
        'ToolBar',           'auto',...
        'HandleVisibility',   'callback',...
        'Resize',            'on',...
        'visible',           'on');
    
    %Create Axes
    Axes=axes(Fig);
    hold(Axes,'on'); grid(Axes,'on'); axis(Axes,'manual')
    Axes.DataAspectRatio=[1,1,1];
    XLim=block.DialogPrm(2).Data; xlim(Axes,XLim);
    YLim=block.DialogPrm(3).Data; ylim(Axes,YLim);
    xlabel(Axes,'[m]'); ylabel(Axes,'[m]');
else %figure was provided in input
    Fig=varargin{1};
    Axes=findobj(Fig,'type','axes');
    cla(Axes);
end

%Initalize Drawing
Theta=block.InputPort(2).Data(1);
Omega=block.InputPort(2).Data(2);
F=block.InputPort(3).Data;
L=block.DialogPrm(4).Data;
g=block.DialogPrm(5).Data;
m=block.DialogPrm(6).Data;
BulbX=L*cos(-pi/2+Theta);
BulbY=L*sin(-pi/2+Theta);

%Normalize to "distanace units"
FNorm=(m*g*tan(Theta))/(3*L);
OmegaNorm=sqrt(g/L)/(3*L);

hBulbTransform=hgtransform(Axes);
T=makehgtform('translate',[BulbX,BulbY,0]);
hBulbTransform.Matrix=T;

hBulb=DrawPlate(hBulbTransform,L/4,'red');
hString=plot(Axes,[0,BulbX],[0,BulbY],'linewidth',3,'color','k');
hVel=quiver(Axes,BulbX,BulbY,(Omega/OmegaNorm)*L*cos(Theta),(Omega/OmegaNorm)*L*sin(Theta),...
    'linewidth',3,'color',[0.8,0.5,0.1],'MaxHeadSize',0.5);
hForce=quiver(Axes,BulbX,BulbY,F/FNorm,0,'linewidth',3,'color',[0.2,0.4,1],'MaxHeadSize',0.5);

%Initalize text for time
xtext=0.9*Axes.XLim(1)+0.1*Axes.XLim(2);
ytext=0.1*Axes.YLim(1)+0.9*Axes.YLim(2);
hTime=text(Axes,xtext,ytext,'');

%Static graphic handles
SensorX=linspace(-L,L,20);
SensorY=-1.3*L*ones(size(SensorX));
plot(Axes,SensorX,SensorY,'linewidth',4,...
    'color',[0.5,0,0.5],'marker','s');

TopX=[-L/4,L/4];
TopY=[0,0];
plot(Axes,TopX,TopY,'linewidth',3,'color','k');

%% Storing handles to "figure" and block "UserData"
UserData.Figure = Fig;
UserData.Axes = Axes;
UserData.hBulbTransform=hBulbTransform;
UserData.hBulb=hBulb;
UserData.hString=hString;
UserData.hVel=hVel;
UserData.hForce=hForce;
UserData.hTime = hTime;

%Store in both figure and block
set(gcbh,'UserData',UserData);
end
function PlateHandle=DrawPlate(Parent,r,FaceColor)
t=linspace(0,2*pi,16);
X=cos(t)*r;
Y=sin(t)*r;
PlateHandle=patch('parent',Parent,'XData',X,'YData',Y,'FaceColor',FaceColor);
end
%% Unused fcns
function Terminate(block)
end
function Start(block)
Enable=block.InputPort(1).Data(1);
if ~Enable, return, end


end
function CheckPrms(block)
  %can check validity of parameters here
end