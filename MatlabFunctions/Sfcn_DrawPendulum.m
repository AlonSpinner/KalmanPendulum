function Sfcn_DrawPendulum_LinearKalman(block)
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
block.NumInputPorts = 5;
block.NumOutputPorts = 0;

%setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;

%Register the properties of the input ports

%Enable
block.InputPort(1).Complexity     ='Real';
block.InputPort(1).DataTypeId     =-1;
block.InputPort(1).Dimensions     =1;
block.InputPort(1).SamplingMode   ='Sample';

%Physics State
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

%Linear Kalman State
%Physics State
block.InputPort(5).Complexity     ='Real';
block.InputPort(5).DataTypeId     =-1;
block.InputPort(5).Dimensions     =2;
block.InputPort(5).SamplingMode   ='Sample';

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

%General inputs
F=block.InputPort(3).Data;
L=block.DialogPrm(4).Data;
g=block.DialogPrm(5).Data;
m=block.DialogPrm(6).Data;
FNorm=block.DialogPrm(7).Data;
OmegaNorm=block.DialogPrm(8).Data;

%Draw Physics
Phys_Theta=block.InputPort(2).Data(1);
Phys_Omega=block.InputPort(2).Data(2);
Phys_BulbX=L*cos(-pi/2+Phys_Theta);
Phys_BulbY=L*sin(-pi/2+Phys_Theta);

Phys_T=makehgtform('translate',[Phys_BulbX,Phys_BulbY,0]);
UserData.Phys_hBulbTransform.Matrix=Phys_T;

UserData.Phys_hString.XData(2)=Phys_BulbX;
UserData.Phys_hString.YData(2)=Phys_BulbY;
UserData.Phys_hVel.XData=Phys_BulbX;
UserData.Phys_hVel.YData=Phys_BulbY;
UserData.Phys_hVel.UData=(Phys_Omega/OmegaNorm)*L*cos(Phys_Theta);
UserData.Phys_hVel.VData=(Phys_Omega/OmegaNorm)*L*sin(Phys_Theta);
UserData.hForce.XData=Phys_BulbX;
UserData.hForce.YData=Phys_BulbY;
UserData.hForce.UData=F/FNorm;

%Draw Kalman
Kal_Theta=block.InputPort(5).Data(1);
Kal_Omega=block.InputPort(5).Data(2);
Kal_BulbX=L*cos(-pi/2+Kal_Theta);
Kal_BulbY=L*sin(-pi/2+Kal_Theta);

Kal_T=makehgtform('translate',[Kal_BulbX,Kal_BulbY,0]);
UserData.Kal_hBulbTransform.Matrix=Kal_T;

UserData.Kal_hString.XData(2)=Kal_BulbX;
UserData.Kal_hString.YData(2)=Kal_BulbY;
UserData.Kal_hVel.XData=Kal_BulbX;
UserData.Kal_hVel.YData=Kal_BulbY;
UserData.Kal_hVel.UData=(Kal_Omega/OmegaNorm)*L*cos(Kal_Theta);
UserData.Kal_hVel.VData=(Kal_Omega/OmegaNorm)*L*sin(Kal_Theta);

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
FNorm=block.DialogPrm(7).Data;
OmegaNorm=block.DialogPrm(8).Data;
BulbX=L*cos(-pi/2+Theta);
BulbY=L*sin(-pi/2+Theta);

%Physics
Phys_hBulbTransform=hgtransform(Axes);
Phys_T=makehgtform('translate',[BulbX,BulbY,0]);
Phys_hBulbTransform.Matrix=Phys_T;
Phys_hBulb=DrawPlate(Phys_hBulbTransform,L/4,'red',2,'-');
Phys_hString=plot(Axes,[0,BulbX],[0,BulbY],'linewidth',3,'color','k','linestyle','-');
Phys_hVel=quiver(Axes,BulbX,BulbY,(Omega/OmegaNorm)*L*cos(Theta),(Omega/OmegaNorm)*L*sin(Theta),...
    'linewidth',3,'color',[0.8,0.5,0.1],'MaxHeadSize',0.5,'linestyle','-');

%Kalman
Kal_hBulbTransform=hgtransform(Axes);
Kal_T=makehgtform('translate',[BulbX,BulbY,0]);
Kal_hBulbTransform.Matrix=Kal_T;
Kal_hBulb=DrawPlate(Kal_hBulbTransform,L/4,'none',2,'--');
Kal_hString=plot(Axes,[0,BulbX],[0,BulbY],'linewidth',3,'color','k','linestyle','--');
Kal_hVel=quiver(Axes,BulbX,BulbY,(Omega/OmegaNorm)*L*cos(Theta),(Omega/OmegaNorm)*L*sin(Theta),...
    'linewidth',3,'color',[0.8,0.5,0.1],'MaxHeadSize',0.5,'linestyle','--');

%Force
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

legend([Kal_hString,Phys_hString],{'Kalman Filter','Physics'})
%% Storing handles to "figure" and block "UserData"
UserData.Figure = Fig;
UserData.Axes = Axes;
UserData.hForce=hForce;
UserData.hTime = hTime;

%Physics
UserData.Phys_hBulbTransform=Phys_hBulbTransform;
UserData.Phys_hBulb=Phys_hBulb;
UserData.Phys_hString=Phys_hString;
UserData.Phys_hVel=Phys_hVel;

%Kalman
UserData.Kal_hBulbTransform=Kal_hBulbTransform;
UserData.Kal_hBulb=Kal_hBulb;
UserData.Kal_hString=Kal_hString;
UserData.Kal_hVel=Kal_hVel;

%Store in both figure and block
set(gcbh,'UserData',UserData);
end
function PlateHandle=DrawPlate(Parent,r,FaceColor,LineWidth,LineStyle)
t=linspace(0,2*pi,16);
X=cos(t)*r;
Y=sin(t)*r;
PlateHandle=patch('parent',Parent,'XData',X,'YData',Y,'FaceColor',FaceColor,...
    'linewidth',LineWidth,'linestyle',LineStyle);
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