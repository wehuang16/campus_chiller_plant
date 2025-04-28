within campus_chiller_plant.Obsolete.Examples;
model Chiller_Storage_CoolingTower_ParallelChillerWithStagingControl
  "Put another chiller"
  package CondensorWater =  Buildings.Media.Water;
  package ChilledWater =  Buildings.Media.Water;

  parameter Modelica.Units.SI.Power P_nominal=-per.QEva_flow_nominal/per.COP_nominal
    "Nominal compressor power (at y=1)";
  parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal=10
    "Temperature difference evaporator inlet-outlet";
  parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal=10
    "Temperature difference condenser outlet-inlet";
  parameter Real COPc_nominal = 3 "Chiller COP";
  parameter Modelica.Units.SI.MassFlowRate mEva_flow_nominal=per.mEva_flow_nominal
    "Nominal mass flow rate at evaporator";
  parameter Modelica.Units.SI.MassFlowRate mCon_flow_nominal=per.mCon_flow_nominal
    "Nominal mass flow rate at condenser";
  parameter Modelica.Units.SI.AbsolutePressure dP0=1280;
  Buildings.Fluid.FixedResistances.PressureDrop res1(
    redeclare package Medium = CondensorWater,
    m_flow_nominal=mCon_flow_nominal,
    dp_nominal=dP0)  "Flow resistance"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-8,-96})));
  parameter
    Buildings.Fluid.Chillers.Data.ElectricEIR.ElectricEIRChiller_McQuay_WSC_471kW_5_89COP_Vanes
    per(
    QEva_flow_nominal(displayUnit="kW") = -1230000,
    COP_nominal=7,
    PLRMax=1,
    PLRMinUnl=0.4,
    mEva_flow_nominal=1000*2*0.06,
    mCon_flow_nominal=1000*3*0.06,
    capFunT={0.70790824,-0.002006568,-0.00259605,0.030058776,-0.0010564344,0.0020457036},
    EIRFunT={0.5605438,-0.01377927,6.57072e-005,0.013219362,0.000268596,-0.0005011308},
    EIRFunPLR={0.17149273,0.58820208,0.23737257}) "Chiller performance data"
    annotation (Placement(transformation(extent={{304,92},{346,134}})));

  Buildings.Fluid.Chillers.ElectricEIR chi1(
    redeclare package Medium1 = CondensorWater,
    redeclare package Medium2 = ChilledWater,
    per=per,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    dp1_nominal=dP0,
    dp2_nominal=dP0) "Chiller model" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={22,-12})));
  Buildings.Fluid.Movers.FlowControlled_m_flow Pump_CHW_Primary1(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    tau=30,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    "Pump for chilled water loop" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={44,-118})));
  Modelica.Blocks.Sources.Constant SP_TCHe[2](k={273.15 + 5,273.15 + 5})
    "chilled water flow rate" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-76,62})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCHW_supply(redeclare package
      Medium = ChilledWater)
    annotation (Placement(transformation(extent={{52,98},{72,118}})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCHWR(redeclare package Medium =
        ChilledWater)
    annotation (Placement(transformation(extent={{220,-94},{240,-74}})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_mS(redeclare package Medium =
        ChilledWater)                                                                   annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={94,-34})));
  Buildings.Fluid.Movers.FlowControlled_m_flow Pump_CHW_Secondary(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    tau=30,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    "Pump for chilled water loop" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={182,20})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_msup(redeclare package Medium =
        ChilledWater)                                                                     annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={208,20})));
  Buildings.Fluid.FixedResistances.PressureDrop res2(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    dp_nominal=dP0)                                                                         annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={240,20})));
  Buildings.Fluid.FixedResistances.Junction junSu(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={mEva_flow_nominal,-0.9*mEva_flow_nominal,-0.1*
        mEva_flow_nominal},
    dp_nominal={0,0,0})                         annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={88,16})));
  Buildings.Fluid.FixedResistances.Junction junRe(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={0.9*mEva_flow_nominal,-mEva_flow_nominal,0.1*
        mEva_flow_nominal},
    dp_nominal={0,0,0})                                                                annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={170,-94})));
  Buildings.Fluid.FixedResistances.PressureDrop res3(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    dp_nominal=dP0)                                                                         annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={172,-38})));
  Buildings.Fluid.Sensors.Pressure Sensor_pSupper(redeclare package Medium =
        ChilledWater) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={110,-2})));
  Buildings.Fluid.Sensors.Pressure Sensor_pSdown(redeclare package Medium =
        ChilledWater)                                                                    annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={196,-62})));
  Buildings.Fluid.Sensors.Pressure Sensor_psub_Suction(redeclare package Medium
      = ChilledWater)
    annotation (Placement(transformation(extent={{130,20},{150,40}})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCHWS(redeclare package Medium =
        ChilledWater)
    annotation (Placement(transformation(extent={{110,22},{130,42}})));
  Modelica.Blocks.Sources.Pulse SP_mCH(
    amplitude=0.9*mEva_flow_nominal,
    width=20,
    period(displayUnit="d") = 86400,
    offset=0.1*mEva_flow_nominal,
    startTime(displayUnit="h") = -18000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={98,-168})));
  Modelica.Blocks.Math.Gain SP_msup_dummys(k=1/(4200*5)) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={522,84})));
  Modelica.Blocks.Sources.Pulse BuildingCoolingLoads(
    amplitude=500*3*1e3,
    width=50,
    period(displayUnit="d") = 86400,
    offset=0.1*500*3*1e3,
    startTime(displayUnit="h") = 25200) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={550,-72})));
  Buildings.Fluid.Chillers.ElectricEIR chi2(
    redeclare package Medium1 = CondensorWater,
    redeclare package Medium2 = ChilledWater,
    per=per,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    dp1_nominal=dP0,
    dp2_nominal=dP0) "Chiller model"
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-10,-12})));
  Buildings.Fluid.HeatExchangers.CoolingTowers.YorkCalc cooTow(
    redeclare package Medium = CondensorWater,
    m_flow_nominal=mCon_flow_nominal,
    PFan_nominal=6000,
    TAirInWB_nominal(displayUnit="degC") = 283.15,
    TApp_nominal=6,
    dp_nominal=3*dP0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial)
    "Cooling tower"                                   annotation (Placement(
        transformation(
        extent={{-29,-30},{29,30}},
        origin={-241,14})));
  Buildings.Fluid.Movers.FlowControlled_m_flow pumCW1(
    redeclare package Medium = CondensorWater,
    m_flow_nominal=mCon_flow_nominal,
    dp(start=dP0),
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    "Condenser water pump" annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={-116,24})));
  Buildings.Fluid.Storage.ExpansionVessel expVesChi(redeclare package Medium =
        CondensorWater, V_start=1)
    annotation (Placement(transformation(extent={{-50,-107},{-30,-87}})));
  Modelica.Blocks.Sources.Constant Twb(k=273.15 + 10) "chilled water flow rate"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-346,26})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_mCW(redeclare package Medium =
        CondensorWater)                                                                 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-174,16})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCWS(redeclare package Medium =
        CondensorWater)
    annotation (Placement(transformation(extent={{-162,42},{-142,62}})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCWR(redeclare package Medium =
        CondensorWater)
    annotation (Placement(transformation(extent={{-168,-106},{-148,-86}})));
  Modelica.Blocks.Math.Gain SP_mCWS(k=mCon_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-126,-72})));
  Modelica.Blocks.Continuous.LimPID PID(yMax=1, yMin=0)
    annotation (Placement(transformation(extent={{-336,56},{-316,76}})));
  Modelica.Blocks.Sources.Constant SP_TCWS(k=273.15 + 10)
                              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-386,46})));
  Buildings.Fluid.Sources.Boundary_pT bou(redeclare package Medium =
        ChilledWater, nPorts=1)
    annotation (Placement(transformation(extent={{262,-136},{282,-116}})));
  Buildings.Fluid.Storage.Stratified tan(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    VTan=100,
    hTan=5,
    dIns=0.01,
    nSeg=20,
    T_start=278.15)
    annotation (Placement(transformation(extent={{132,-52},{158,-26}})));
  Buildings.Fluid.Sensors.Temperature Sensor_Ttankin(redeclare package Medium
      = ChilledWater)
    annotation (Placement(transformation(extent={{222,-24},{242,-4}})));
  Buildings.Fluid.Sensors.Temperature Sensor_Ttankout(redeclare package Medium
      = ChilledWater)
    annotation (Placement(transformation(extent={{118,-74},{138,-54}})));
  campus_chiller_plant.Examples.BaseClasses.chiller_tes_plant_controller_Donghun
    chiller_tes_plant_controller_D
    annotation (Placement(transformation(extent={{170,184},{190,204}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.TimeTable intTimTab(
    table=[0,1; 8,2; 16,3; 21,1; 24,1],
    timeScale=3600,
    period=86400)
    annotation (Placement(transformation(extent={{34,182},{54,202}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensorTop
    annotation (Placement(transformation(extent={{226,176},{246,196}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensorBottom
    annotation (Placement(transformation(extent={{226,140},{246,160}})));
  campus_chiller_plant.Examples.BaseClasses.TesStatusController tesStatusController
    annotation (Placement(transformation(extent={{358,148},{378,168}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis largeLoadProtection(uLow=273.15
         + 23, uHigh=273.15 + 28)
    annotation (Placement(transformation(extent={{428,180},{448,200}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis hys(uLow=273.15 + 15, uHigh=
        273.15 + 22)
    annotation (Placement(transformation(extent={{424,138},{444,158}})));
  Buildings.Controls.OBC.CDL.Reals.Switch loaAct "actual load"
    annotation (Placement(transformation(extent={{540,-22},{560,-2}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant const(k=0)
    annotation (Placement(transformation(extent={{450,-12},{470,8}})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai(k=mEva_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={162,-138})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai1(k=mEva_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={240,94})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow fixHeaFlo(T_ref=
        293.15)
    "Fixed heat flow rate"
    annotation (Placement(transformation(extent={{344,-16},{364,4}})));
  Buildings.Fluid.MixingVolumes.MixingVolume vol(
    nPorts=4,
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    V=20,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Mixing volume"
    annotation (Placement(transformation(extent={{380,-38},{400,-18}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=vol.heatPort.T)
    annotation (Placement(transformation(extent={{270,-78},{290,-58}})));
  Modelica.Blocks.Sources.Sine loaVar(
    amplitude=500000,
    f=1/86400,
    phase=3.1415926535898,
    offset=500000,
    startTime(displayUnit="h") = 0)     "Variable demand load"
    annotation (Placement(transformation(extent={{480,-108},{500,-88}})));
  Buildings.DHC.Plants.Cooling.Controls.ChillerStage chiStaCon(
    redeclare package Medium = ChilledWater,
    final tWai=300,
    final QChi_nominal=per.QEva_flow_nominal)
    "Chiller staging controller"
    annotation (Placement(transformation(extent={{-202,52},{-182,72}})));
  Buildings.Fluid.Movers.FlowControlled_m_flow pumCW2(
    redeclare package Medium = CondensorWater,
    m_flow_nominal=mCon_flow_nominal,
    dp(start=dP0),
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    "Condenser water pump" annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={-112,-26})));
  Buildings.Fluid.FixedResistances.Junction junConv(
    redeclare package Medium = CondensorWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-8,-58})));
  Buildings.Fluid.FixedResistances.Junction junDiv(
    redeclare package Medium = CondensorWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-142,-2})));
  campus_chiller_plant.Examples.BaseClasses.ChillerStagingDataProcessing chillerStagingDataProcessing
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-162,-50})));
  Buildings.Fluid.FixedResistances.Junction junDiv1(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={80,-104})));
  Buildings.Fluid.Movers.FlowControlled_m_flow Pump_CHW_Primary2(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    tau=30,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    "Pump for chilled water loop" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={44,-72})));
  campus_chiller_plant.Examples.BaseClasses.ChillerStagingDataProcessing chillerStagingDataProcessing1
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={6,-156})));
  Buildings.Fluid.FixedResistances.Junction junConv1(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={24,78})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCHW_return(redeclare package
      Medium = ChilledWater)
    annotation (Placement(transformation(extent={{104,-88},{124,-68}})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_mCHi(redeclare package Medium =
        ChilledWater)                                                                     annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={114,-118})));
equation
  connect(Pump_CHW_Secondary.port_b, Sensor_msup.port_a)
    annotation (Line(points={{192,20},{198,20}}, color={0,127,255}));
  connect(Sensor_msup.port_b, res2.port_a)
    annotation (Line(points={{218,20},{230,20}},         color={0,127,255}));
  connect(junSu.port_3, Sensor_mS.port_a)
    annotation (Line(points={{88,6},{88,-10},{84,-10},{84,-24},{94,-24}},
                                                        color={0,127,255}));
  connect(res3.port_b, junRe.port_3)
    annotation (Line(points={{172,-48},{170,-52},{170,-84}},
                                                 color={0,127,255}));
  connect(junSu.port_3, Sensor_pSupper.port) annotation (Line(points={{88,6},{
          88,-2},{100,-2}},      color={0,127,255}));
  connect(res3.port_b, Sensor_pSdown.port)
    annotation (Line(points={{172,-48},{172,-62},{186,-62}},
                                                   color={0,127,255}));
  connect(junSu.port_2, Sensor_psub_Suction.port)
    annotation (Line(points={{98,16},{128,16},{128,14},{140,14},{140,20}},
                                               color={0,127,255}));
  connect(junSu.port_2, Sensor_TCHWS.port)
    annotation (Line(points={{98,16},{120,16},{120,22}},
                                               color={0,127,255}));
  connect(SP_TCHe[2].y, chi2.TSet)
    annotation (Line(points={{-65,62},{-7,62},{-7,0}}, color={0,0,127}));
  connect(SP_TCHe[1].y, chi1.TSet)
    annotation (Line(points={{-65,62},{-8,62},{-8,8},{25,8},{25,0}},
                                                         color={0,0,127}));
  connect(res1.port_b, expVesChi.port_a) annotation (Line(points={{-8,-106},{
          -40,-106},{-40,-107}}, color={0,127,255}));
  connect(Twb.y, cooTow.TAir)
    annotation (Line(points={{-335,26},{-275.8,26}}, color={0,0,127}));
  connect(expVesChi.port_a, cooTow.port_a) annotation (Line(points={{-40,-107},
          {-188,-107},{-188,-108},{-324,-108},{-324,14},{-270,14}}, color={0,
          127,255}));
  connect(Sensor_mCW.port_b, Sensor_TCWS.port)
    annotation (Line(points={{-164,16},{-152,16},{-152,42}},
                                                 color={0,127,255}));
  connect(expVesChi.port_a, Sensor_TCWR.port) annotation (Line(points={{-40,
          -107},{-99,-107},{-99,-106},{-158,-106}}, color={0,127,255}));
  connect(PID.y, cooTow.y) annotation (Line(points={{-315,66},{-302,66},{-302,
          38},{-275.8,38}}, color={0,0,127}));
  connect(SP_TCWS.y, PID.u_m)
    annotation (Line(points={{-375,46},{-326,46},{-326,54}}, color={0,0,127}));
  connect(Sensor_TCWS.T, PID.u_s) annotation (Line(points={{-145,52},{-136,52},
          {-136,88},{-352,88},{-352,66},{-338,66}},
                                                  color={0,0,127}));
  connect(tan.port_a, Sensor_Ttankin.port) annotation (Line(points={{145,-26},{
          145,-16},{212,-16},{212,-24},{216,-24},{216,-32},{232,-32},{232,-24}},
                               color={0,127,255}));
  connect(tan.port_b, Sensor_Ttankout.port) annotation (Line(points={{145,-52},
          {145,-74},{128,-74}}, color={0,127,255}));
  connect(Sensor_psub_Suction.port, Pump_CHW_Secondary.port_a) annotation (Line(
        points={{140,20},{140,14},{166,14},{166,20},{172,20}}, color={0,127,255}));
  connect(intTimTab.y[1], chiller_tes_plant_controller_D.systemCommand)
    annotation (Line(points={{56,192},{120,192},{120,199.4},{168,199.4}}, color
        ={255,127,0}));
  connect(temperatureSensorTop.T,tesStatusController. TesTopTemp) annotation (
      Line(points={{247,186},{346,186},{346,161.8},{355.8,161.8}},
                                                              color={0,0,127}));
  connect(temperatureSensorBottom.T,tesStatusController. TesBottomTemp)
    annotation (Line(points={{247,150},{252,150},{252,151.8},{356,151.8}},
                                                                color={0,0,127}));
  connect(temperatureSensorTop.port, tan.heaPorVol[3]) annotation (Line(points=
          {{226,186},{210,186},{210,154},{162,154},{162,-39.2925},{145,-39.2925}},
        color={191,0,0}));
  connect(temperatureSensorBottom.port, tan.heaPorVol[17]) annotation (Line(
        points={{226,150},{145,150},{145,-38.7465}}, color={191,0,0}));
  connect(tesStatusController.TesMode, chiller_tes_plant_controller_D.tesStatus)
    annotation (Line(points={{380,158},{402,158},{402,128},{128,128},{128,187},
          {168,187}}, color={255,127,0}));
  connect(const.y, loaAct.u1) annotation (Line(points={{472,-2},{530,-2},{530,
          -4},{538,-4}}, color={0,0,127}));
  connect(largeLoadProtection.y, loaAct.u2) annotation (Line(points={{450,190},
          {456,190},{456,16},{444,16},{444,-20},{528,-20},{528,-12},{538,-12}},
        color={255,0,255}));
  connect(loaAct.y, SP_msup_dummys.u) annotation (Line(points={{562,-12},{572,
          -12},{572,84},{534,84}},                 color={0,0,127}));
  connect(hys.y, chiller_tes_plant_controller_D.loadRequest) annotation (Line(
        points={{446,148},{466,148},{466,146},{110,146},{110,194},{168,194}},
        color={255,0,255}));
  connect(Sensor_mS.port_b, tan.port_b)
    annotation (Line(points={{94,-44},{94,-52},{145,-52}}, color={0,127,255}));
  connect(res3.port_a, tan.port_a) annotation (Line(points={{172,-28},{172,-26},
          {145,-26}}, color={0,127,255}));
  connect(chiller_tes_plant_controller_D.chillerPumpSpeed, gai.u) annotation (
      Line(points={{192,195.8},{198,195.8},{198,-132},{174,-132},{174,-138}},
        color={0,0,127}));
  connect(chiller_tes_plant_controller_D.tesPumpSpeed, gai1.u) annotation (Line(
        points={{192,186.2},{196,186.2},{196,160},{140,160},{140,96},{220,96},{
          220,112},{260,112},{260,94},{252,94}}, color={0,0,127}));
  connect(gai1.y, Pump_CHW_Secondary.m_flow_in) annotation (Line(points={{228,
          94},{220,94},{220,64},{182,64},{182,32}}, color={0,0,127}));
  connect(fixHeaFlo.port,vol.heatPort)
    annotation (Line(points={{364,-6},{364,-8},{372,-8},{372,-28},{380,-28}},
                                                              color={191,0,0}));
  connect(loaAct.y, fixHeaFlo.Q_flow) annotation (Line(points={{562,-12},{566,
          -12},{566,-48},{332,-48},{332,-6},{344,-6}}, color={0,0,127}));
  connect(res2.port_b, vol.ports[1]) annotation (Line(points={{250,20},{270,20},
          {270,14},{286,14},{286,-38},{388.5,-38}}, color={0,127,255}));
  connect(vol.ports[2], junRe.port_1) annotation (Line(points={{389.5,-38},{
          389.5,-40},{392,-40},{392,-100},{320,-100},{320,-84},{300,-84},{300,
          -88},{280,-88},{280,-96},{248,-96},{248,-104},{188,-104},{188,-94},{
          180,-94}}, color={0,127,255}));
  connect(Sensor_TCHWR.port, vol.ports[3]) annotation (Line(points={{230,-94},{
          296,-94},{296,-98},{390.5,-98},{390.5,-38}}, color={0,127,255}));
  connect(realExpression.y, hys.u) annotation (Line(points={{291,-68},{364,-68},
          {364,-76},{424,-76},{424,148},{422,148}}, color={0,0,127}));
  connect(hys.u, largeLoadProtection.u) annotation (Line(points={{422,148},{416,
          148},{416,154},{412,154},{412,190},{426,190}}, color={0,0,127}));
  connect(bou.ports[1], vol.ports[4]) annotation (Line(points={{282,-126},{328,
          -126},{328,-128},{391.5,-128},{391.5,-38}}, color={0,127,255}));
  connect(loaVar.y, loaAct.u3) annotation (Line(points={{501,-98},{501,-100},{
          512,-100},{512,-24},{532,-24},{532,-20},{538,-20}}, color={0,0,127}));
  connect(cooTow.port_b, Sensor_mCW.port_a) annotation (Line(points={{-212,14},
          {-212,16},{-184,16}}, color={0,127,255}));
  connect(Sensor_mCW.port_b, junDiv.port_1) annotation (Line(points={{-164,16},
          {-164,8},{-152,8},{-152,-2}}, color={0,127,255}));
  connect(junDiv.port_2, pumCW1.port_a) annotation (Line(points={{-132,-2},{
          -132,8},{-126,8},{-126,24}}, color={0,127,255}));
  connect(junDiv.port_3, pumCW2.port_a) annotation (Line(points={{-142,-12},{
          -142,-26},{-122,-26}}, color={0,127,255}));
  connect(SP_mCWS.y, chillerStagingDataProcessing.setpoint) annotation (Line(
        points={{-126,-61},{-124,-62},{-155.4,-62}}, color={0,0,127}));
  connect(chiStaCon.y, chillerStagingDataProcessing.stagingCommand) annotation (
     Line(points={{-181.375,62},{-178,62},{-178,64},{-180,64},{-180,-62},{
          -167.8,-62}}, color={255,0,255}));
  connect(chillerStagingDataProcessing.setpointOutput[1], pumCW1.m_flow_in)
    annotation (Line(points={{-161.7,-38},{-161.7,-28},{-148,-28},{-148,-32},{
          -132,-32},{-132,-52},{-88,-52},{-88,0},{-116,0},{-116,12}}, color={0,
          0,127}));
  connect(chillerStagingDataProcessing.setpointOutput[2], pumCW2.m_flow_in)
    annotation (Line(points={{-162.7,-38},{-162.7,-36},{-160,-36},{-160,-28},{
          -148,-28},{-148,-32},{-132,-32},{-132,-52},{-112,-52},{-112,-38}},
        color={0,0,127}));
  connect(pumCW1.port_b, chi1.port_a1)
    annotation (Line(points={{-106,24},{16,24},{16,-2}}, color={0,127,255}));
  connect(pumCW2.port_b, chi2.port_a1) annotation (Line(points={{-102,-26},{-78,
          -26},{-78,-18},{-16,-18},{-16,-2}}, color={0,127,255}));
  connect(junConv.port_1, chi1.port_b1) annotation (Line(points={{-8,-48},{-8,
          -32},{16,-32},{16,-22}}, color={0,127,255}));
  connect(junConv.port_3, chi2.port_b1) annotation (Line(points={{-18,-58},{-18,
          -22},{-16,-22}}, color={0,127,255}));
  connect(junConv.port_2, res1.port_a)
    annotation (Line(points={{-8,-68},{-8,-86}}, color={0,127,255}));
  connect(gai.u, SP_mCWS.u) annotation (Line(points={{174,-138},{178,-138},{178,
          -188},{-126,-188},{-126,-84}}, color={0,0,127}));
  connect(junDiv1.port_2, Pump_CHW_Primary1.port_a) annotation (Line(points={{
          70,-104},{64,-104},{64,-118},{54,-118}}, color={0,127,255}));
  connect(junDiv1.port_3, Pump_CHW_Primary2.port_a) annotation (Line(points={{
          80,-94},{65,-94},{65,-72},{54,-72}}, color={0,127,255}));
  connect(gai.y, chillerStagingDataProcessing1.setpoint) annotation (Line(
        points={{150,-138},{28,-138},{28,-176},{12.6,-176},{12.6,-168}}, color=
          {0,0,127}));
  connect(chillerStagingDataProcessing1.setpointOutput[1], Pump_CHW_Primary1.m_flow_in)
    annotation (Line(points={{6.3,-144},{6.3,-140},{24,-140},{24,-136},{44,-136},
          {44,-130}}, color={0,0,127}));
  connect(chillerStagingDataProcessing1.setpointOutput[2], Pump_CHW_Primary2.m_flow_in)
    annotation (Line(points={{5.3,-144},{5.3,-140},{24,-140},{24,-92},{44,-92},
          {44,-84}}, color={0,0,127}));
  connect(chiStaCon.y, chillerStagingDataProcessing1.stagingCommand)
    annotation (Line(points={{-181.375,62},{-174,62},{-174,24},{-216,24},{-216,
          -168},{0.2,-168}}, color={255,0,255}));
  connect(Pump_CHW_Primary2.port_b, chi2.port_a2) annotation (Line(points={{34,
          -72},{28,-72},{28,-70},{-4,-70},{-4,-22}}, color={0,127,255}));
  connect(Pump_CHW_Primary1.port_b, chi1.port_a2) annotation (Line(points={{34,
          -118},{28,-118},{28,-22}}, color={0,127,255}));
  connect(chi1.port_b2, junConv1.port_1) annotation (Line(points={{28,-2},{34,
          -2},{34,20},{8,20},{8,78},{14,78}}, color={0,127,255}));
  connect(junConv1.port_3, chi2.port_b2) annotation (Line(points={{24,68},{24,
          36},{-4,36},{-4,-2}}, color={0,127,255}));
  connect(junConv1.port_2, junSu.port_1) annotation (Line(points={{34,78},{52,
          78},{52,76},{78,76},{78,16}}, color={0,127,255}));
  connect(junConv1.port_2, Sensor_TCHW_supply.port) annotation (Line(points={{
          34,78},{52,78},{52,76},{62,76},{62,98}}, color={0,127,255}));
  connect(junRe.port_2, Sensor_TCHW_return.port) annotation (Line(points={{160,
          -94},{160,-96},{114,-96},{114,-88}}, color={0,127,255}));
  connect(junRe.port_2, Sensor_mCHi.port_a) annotation (Line(points={{160,-94},
          {160,-96},{132,-96},{132,-118},{124,-118}}, color={0,127,255}));
  connect(Sensor_mCHi.port_b, junDiv1.port_1) annotation (Line(points={{104,
          -118},{104,-116},{90,-116},{90,-104}}, color={0,127,255}));
  connect(chiStaCon.y[1], chi1.on) annotation (Line(points={{-181.375,61.8438},
          {-78,61.8438},{-78,36},{19,36},{19,0}}, color={255,0,255}));
  connect(chiStaCon.y[2], chi2.on) annotation (Line(points={{-181.375,62.1562},
          {-172,62.1562},{-172,36},{-136,36},{-136,44},{-84,44},{-84,12},{-13,
          12},{-13,0}}, color={255,0,255}));
  connect(chiller_tes_plant_controller_D.chillerOn, chiStaCon.on) annotation (
      Line(points={{192,200},{206,200},{206,232},{-203.25,232},{-203.25,65.75}},
        color={255,0,255}));
  connect(Sensor_TCHW_supply.T, chiStaCon.TChiWatSup) annotation (Line(points={
          {69,108},{94,108},{94,114},{-228,114},{-228,59.5},{-203.25,59.5}},
        color={0,0,127}));
  connect(Sensor_TCHW_return.T, chiStaCon.TChiWatRet) annotation (Line(points={
          {121,-78},{142,-78},{142,-224},{-212,-224},{-212,61.75},{-203.25,
          61.75}}, color={0,0,127}));
  connect(Sensor_mCHi.m_flow, chiStaCon.mFloChiWat) annotation (Line(points={{
          114,-129},{114,-204},{-203.25,-204},{-203.25,57.25}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-400,
            -160},{380,120}}),
                         graphics={Text(
          extent={{-84,38},{106,-92}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.None,
          textString="CHL")}),                                   Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-400,-160},{380,
            120}}),
        graphics={Text(
          extent={{-114,126},{-54,80}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.None,
          textString="Norminal Cap: 1000 ton
Norminal Water Flows: 2gpm/ton for evap, 3 gpm /to for comp (~5oC DTW).")}),
    experiment(
      StopTime=172800,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end Chiller_Storage_CoolingTower_ParallelChillerWithStagingControl;
