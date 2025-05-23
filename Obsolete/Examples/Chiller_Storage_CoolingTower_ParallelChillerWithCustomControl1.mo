within campus_chiller_plant.Obsolete.Examples;
model Chiller_Storage_CoolingTower_ParallelChillerWithCustomControl1
  "Put another chiller"
  package CondensorWater =  Buildings.Media.Water;
  package ChilledWater =  Buildings.Media.Water;

  parameter Modelica.Units.SI.Power P_nominal=-per1.QEva_flow_nominal/per1.COP_nominal
    "Nominal compressor power (at y=1)";
  parameter Modelica.Units.SI.TemperatureDifference dTEva_nominal=10
    "Temperature difference evaporator inlet-outlet";
  parameter Modelica.Units.SI.TemperatureDifference dTCon_nominal=10
    "Temperature difference condenser outlet-inlet";
  parameter Real COPc_nominal = 3 "Chiller COP";
  parameter Modelica.Units.SI.MassFlowRate mEva_flow_nominal=per1.mEva_flow_nominal
    "Nominal mass flow rate at evaporator";
  parameter Modelica.Units.SI.MassFlowRate mCon_flow_nominal=per1.mCon_flow_nominal
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
    per1(
    QEva_flow_nominal(displayUnit="kW") = -2989000,
    COP_nominal=7,
    PLRMax=1,
    PLRMinUnl=0.4,
    mEva_flow_nominal=100,
    mCon_flow_nominal=150,
    TEvaLvg_nominal=278.15,
    capFunT={0.70790824,-0.002006568,-0.00259605,0.030058776,-0.0010564344,0.0020457036},
    EIRFunT={0.5605438,-0.01377927,6.57072e-005,0.013219362,0.000268596,-0.0005011308},
    EIRFunPLR={0.17149273,0.58820208,0.23737257},
    TEvaLvgMin=278.15,
    TEvaLvgMax=283.71) "Chiller performance data"
    annotation (Placement(transformation(extent={{304,92},{346,134}})));

  Buildings.Fluid.Chillers.ElectricEIR chi1(
    redeclare package Medium1 = CondensorWater,
    redeclare package Medium2 = ChilledWater,
    per=per1,
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
    "chilled water suppy temperature setpoint"
                              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-76,62})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCHW_primary_supply(redeclare
      package Medium = ChilledWater)
    annotation (Placement(transformation(extent={{52,98},{72,118}})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCHWR(redeclare package Medium =
        ChilledWater)
    annotation (Placement(transformation(extent={{408,-84},{428,-64}})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_mS(redeclare package Medium =
        ChilledWater)                                                                   annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={184,-22})));
  Buildings.Fluid.Movers.FlowControlled_m_flow Pump_CHW_Secondary(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    tau=30,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    "Pump for chilled water loop" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={350,32})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_msup(redeclare package Medium =
        ChilledWater)                                                                     annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={396,30})));
  Buildings.Fluid.FixedResistances.PressureDrop res2(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    dp_nominal=dP0)                                                                         annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={428,30})));
  Buildings.Fluid.FixedResistances.Junction junSecSup(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={mEva_flow_nominal,-0.9*mEva_flow_nominal,-0.1*
        mEva_flow_nominal},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={178,28})));
  Buildings.Fluid.FixedResistances.Junction junSecRet(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={0.9*mEva_flow_nominal,-mEva_flow_nominal,0.1*
        mEva_flow_nominal},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={338,-82})));
  Buildings.Fluid.FixedResistances.PressureDrop res3(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    dp_nominal=dP0)                                                                         annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={340,-26})));
  Buildings.Fluid.Sensors.Pressure Sensor_pSupper(redeclare package Medium =
        ChilledWater) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={246,0})));
  Buildings.Fluid.Sensors.Pressure Sensor_pSdown(redeclare package Medium =
        ChilledWater)                                                                    annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={384,-52})));
  Buildings.Fluid.Sensors.Pressure Sensor_psub_Suction(redeclare package Medium
      = ChilledWater)
    annotation (Placement(transformation(extent={{220,32},{240,52}})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCHWS(redeclare package Medium =
        ChilledWater)
    annotation (Placement(transformation(extent={{200,34},{220,54}})));
  Buildings.Fluid.Chillers.ElectricEIR chi2(
    redeclare package Medium1 = CondensorWater,
    redeclare package Medium2 = ChilledWater,
    per=per2,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    dp1_nominal=dP0,
    dp2_nominal=dP0) "Chiller model" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
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
  Modelica.Blocks.Math.Gain gai3(k=mCon_flow_nominal) annotation (Placement(
        transformation(
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
    annotation (Placement(transformation(extent={{450,-126},{470,-106}})));
  Buildings.Fluid.Storage.Stratified tan(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    VTan=2650,
    hTan=8.4352,
    dIns=0.01,
    nSeg=20,
    T_start=278.15)
    annotation (Placement(transformation(extent={{-13,-13},{13,13}},
        rotation=0,
        origin={299,-35})));
  Buildings.Fluid.Sensors.Temperature Sensor_Ttankin(redeclare package Medium
      = ChilledWater)
    annotation (Placement(transformation(extent={{410,-14},{430,6}})));
  Buildings.Fluid.Sensors.Temperature Sensor_Ttankout(redeclare package Medium
      = ChilledWater)
    annotation (Placement(transformation(extent={{244,-48},{264,-28}})));
  campus_chiller_plant.Examples.BaseClasses.chiller_tes_plant_controller_ParallelChillerWithCustomControl
    chiller_tes_plant_controller
    annotation (Placement(transformation(extent={{170,184},{190,204}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.TimeTable intTimTab(
    table=[0,2; 6,1; 14,3; 22,2; 24,2],
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
    annotation (Placement(transformation(extent={{728,-12},{748,8}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant const(k=0)
    annotation (Placement(transformation(extent={{638,-2},{658,18}})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai(k=mEva_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={164,-138})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai1(k=mEva_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={332,70})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow fixHeaFlo(T_ref=
        293.15)
    "Fixed heat flow rate"
    annotation (Placement(transformation(extent={{532,-6},{552,14}})));
  Buildings.Fluid.MixingVolumes.MixingVolume vol(
    nPorts=4,
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    V=200,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Mixing volume"
    annotation (Placement(transformation(extent={{568,-28},{588,-8}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=vol.heatPort.T)
    annotation (Placement(transformation(extent={{458,-68},{478,-48}})));
  Modelica.Blocks.Sources.Sine loaVar(
    amplitude=500000,
    f=1/86400,
    phase=4.1887902047864,
    offset=500000,
    startTime(displayUnit="h") = 0)     "Variable demand load"
    annotation (Placement(transformation(extent={{668,-100},{688,-80}})));
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
  Buildings.Fluid.FixedResistances.Junction junCwPumSup(
    redeclare package Medium = CondensorWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-8,-58})));
  Buildings.Fluid.FixedResistances.Junction junCwPumRet(
    redeclare package Medium = CondensorWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-142,-2})));
  Buildings.Fluid.FixedResistances.Junction junChwPumRet(
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
  Buildings.Fluid.FixedResistances.Junction junChwPumSup(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={24,78})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCHW_return(redeclare package
      Medium = ChilledWater)
    annotation (Placement(transformation(extent={{268,-108},{288,-88}})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_mCHi(redeclare package Medium =
        ChilledWater)                                                                     annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={114,-118})));
  Buildings.Fluid.Movers.FlowControlled_m_flow Pump_CHW_Bypass(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    tau=30,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    "Pump for chilled water bypass" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={110,-54})));
  Buildings.Fluid.FixedResistances.Junction junPriSup(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={100,70})));
  Buildings.Fluid.FixedResistances.Junction junPriRet(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={162,-92})));
  parameter
    Buildings.Fluid.Chillers.Data.ElectricEIR.ElectricEIRChiller_McQuay_WSC_471kW_5_89COP_Vanes
    per2(
    QEva_flow_nominal(displayUnit="kW") = -3165000,
    COP_nominal=7,
    PLRMax=1,
    PLRMinUnl=0.4,
    mEva_flow_nominal=100,
    mCon_flow_nominal=150,
    TEvaLvg_nominal=278.15,
    capFunT={0.70790824,-0.002006568,-0.00259605,0.030058776,-0.0010564344,0.0020457036},
    EIRFunT={0.5605438,-0.01377927,6.57072e-005,0.013219362,0.000268596,-0.0005011308},
    EIRFunPLR={0.17149273,0.58820208,0.23737257},
    TEvaLvgMin=278.15,
    TEvaLvgMax=283.71) "Chiller performance data"
    annotation (Placement(transformation(extent={{400,68},{442,110}})));

  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai2(k=18)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={152,-54})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={198,-58})));
  Buildings.Controls.OBC.CDL.Reals.Switch swi
    annotation (Placement(transformation(extent={{226,-160},{246,-140}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con(k=0)
    annotation (Placement(transformation(extent={{194,-192},{214,-172}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea1(realTrue=0.5)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={62,-38})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai4(k=mEva_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={78,-82})));
  Modelica.Blocks.Math.Gain gai5(k=mCon_flow_nominal) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-88,-68})));
  Buildings.Controls.OBC.CDL.Reals.Switch swi1
    annotation (Placement(transformation(extent={{268,68},{288,88}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con1(k=0)
    annotation (Placement(transformation(extent={{252,36},{272,56}})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCHW_primary_return(redeclare
      package Medium = ChilledWater)
    annotation (Placement(transformation(extent={{72,-176},{92,-156}})));
equation
  connect(Pump_CHW_Secondary.port_b, Sensor_msup.port_a)
    annotation (Line(points={{360,32},{360,30},{386,30}},
                                                 color={0,127,255}));
  connect(Sensor_msup.port_b, res2.port_a)
    annotation (Line(points={{406,30},{418,30}},         color={0,127,255}));
  connect(junSecSup.port_3, Sensor_mS.port_a)
    annotation (Line(points={{178,18},{184,18},{184,-12}}, color={0,127,255}));
  connect(res3.port_b, junSecRet.port_3) annotation (Line(points={{340,-36},{
          338,-40},{338,-72}}, color={0,127,255}));
  connect(junSecSup.port_3, Sensor_pSupper.port) annotation (Line(points={{178,
          18},{184,18},{184,0},{236,0}}, color={0,127,255}));
  connect(res3.port_b, Sensor_pSdown.port)
    annotation (Line(points={{340,-36},{340,-52},{374,-52}},
                                                   color={0,127,255}));
  connect(junSecSup.port_2, Sensor_psub_Suction.port) annotation (Line(points={
          {188,28},{200,28},{200,32},{208,32},{208,28},{220,28},{220,24},{230,
          24},{230,32}}, color={0,127,255}));
  connect(junSecSup.port_2, Sensor_TCHWS.port) annotation (Line(points={{188,28},
          {200,28},{200,34},{210,34}}, color={0,127,255}));
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
  connect(tan.port_a, Sensor_Ttankin.port) annotation (Line(points={{299,-22},{
          299,-8},{400,-8},{400,-24},{420,-24},{420,-14}},
                               color={0,127,255}));
  connect(tan.port_b, Sensor_Ttankout.port) annotation (Line(points={{299,-48},
          {299,-60},{254,-60},{254,-48}},
                                color={0,127,255}));
  connect(Sensor_psub_Suction.port, Pump_CHW_Secondary.port_a) annotation (Line(
        points={{230,32},{230,24},{328,24},{328,32},{340,32}}, color={0,127,255}));
  connect(intTimTab.y[1], chiller_tes_plant_controller.systemCommand)
    annotation (Line(points={{56,192},{120,192},{120,201.8},{168,201.8}}, color
        ={255,127,0}));
  connect(temperatureSensorTop.T,tesStatusController. TesTopTemp) annotation (
      Line(points={{247,186},{346,186},{346,161.8},{355.8,161.8}},
                                                              color={0,0,127}));
  connect(temperatureSensorBottom.T,tesStatusController. TesBottomTemp)
    annotation (Line(points={{247,150},{252,150},{252,151.8},{356,151.8}},
                                                                color={0,0,127}));
  connect(temperatureSensorTop.port, tan.heaPorVol[3]) annotation (Line(points={{226,186},
          {220,186},{220,116},{292,116},{292,-16},{280,-16},{280,-35.2925},{299,
          -35.2925}},
        color={191,0,0}));
  connect(temperatureSensorBottom.port, tan.heaPorVol[17]) annotation (Line(
        points={{226,150},{220,150},{220,116},{292,116},{292,-16},{280,-16},{
          280,-34.7465},{299,-34.7465}},             color={191,0,0}));
  connect(tesStatusController.TesMode, chiller_tes_plant_controller.tesStatus)
    annotation (Line(points={{380,158},{402,158},{402,128},{128,128},{128,191.6},
          {167.8,191.6}}, color={255,127,0}));
  connect(const.y, loaAct.u1) annotation (Line(points={{660,8},{664,6},{726,6}},
                         color={0,0,127}));
  connect(largeLoadProtection.y, loaAct.u2) annotation (Line(points={{450,190},
          {688,190},{688,-2},{726,-2}},
        color={255,0,255}));
  connect(hys.y, chiller_tes_plant_controller.loadRequest) annotation (Line(
        points={{446,148},{466,148},{466,146},{110,146},{110,197.8},{168,197.8}},
        color={255,0,255}));
  connect(Sensor_mS.port_b, tan.port_b)
    annotation (Line(points={{184,-32},{184,-40},{232,-40},{232,-56},{256,-56},
          {256,-60},{299,-60},{299,-48}},                  color={0,127,255}));
  connect(res3.port_a, tan.port_a) annotation (Line(points={{340,-16},{340,-8},
          {299,-8},{299,-22}},
                      color={0,127,255}));
  connect(fixHeaFlo.port,vol.heatPort)
    annotation (Line(points={{552,4},{560,4},{560,-18},{568,-18}},
                                                              color={191,0,0}));
  connect(loaAct.y, fixHeaFlo.Q_flow) annotation (Line(points={{750,-2},{760,-2},
          {760,32},{528,32},{528,24},{520,24},{520,4},{532,4}},
                                                       color={0,0,127}));
  connect(res2.port_b, vol.ports[1]) annotation (Line(points={{438,30},{524,30},
          {524,-40},{576.5,-40},{576.5,-28}},       color={0,127,255}));
  connect(vol.ports[2], junSecRet.port_1) annotation (Line(points={{577.5,-28},
          {577.5,-40},{500,-40},{500,-92},{360,-92},{360,-82},{348,-82}}, color
        ={0,127,255}));
  connect(Sensor_TCHWR.port, vol.ports[3]) annotation (Line(points={{418,-84},{
          418,-92},{578.5,-92},{578.5,-28}},           color={0,127,255}));
  connect(realExpression.y, hys.u) annotation (Line(points={{479,-58},{479,-60},
          {488,-60},{488,132},{416,132},{416,148},{422,148}},
                                                    color={0,0,127}));
  connect(hys.u, largeLoadProtection.u) annotation (Line(points={{422,148},{416,
          148},{416,154},{412,154},{412,190},{426,190}}, color={0,0,127}));
  connect(bou.ports[1], vol.ports[4]) annotation (Line(points={{470,-116},{
          579.5,-116},{579.5,-28}},                   color={0,127,255}));
  connect(loaVar.y, loaAct.u3) annotation (Line(points={{689,-90},{689,-92},{
          716,-92},{716,-10},{726,-10}},                      color={0,0,127}));
  connect(cooTow.port_b, Sensor_mCW.port_a) annotation (Line(points={{-212,14},
          {-212,16},{-184,16}}, color={0,127,255}));
  connect(Sensor_mCW.port_b, junCwPumRet.port_1) annotation (Line(points={{-164,
          16},{-164,8},{-152,8},{-152,-2}}, color={0,127,255}));
  connect(junCwPumRet.port_2, pumCW1.port_a) annotation (Line(points={{-132,-2},
          {-132,8},{-126,8},{-126,24}}, color={0,127,255}));
  connect(junCwPumRet.port_3, pumCW2.port_a) annotation (Line(points={{-142,-12},
          {-142,-26},{-122,-26}}, color={0,127,255}));
  connect(pumCW1.port_b, chi1.port_a1)
    annotation (Line(points={{-106,24},{16,24},{16,-2}}, color={0,127,255}));
  connect(pumCW2.port_b, chi2.port_a1) annotation (Line(points={{-102,-26},{-78,
          -26},{-78,-18},{-16,-18},{-16,-2}}, color={0,127,255}));
  connect(junCwPumSup.port_1, chi1.port_b1) annotation (Line(points={{-8,-48},{
          -8,-32},{16,-32},{16,-22}}, color={0,127,255}));
  connect(junCwPumSup.port_3, chi2.port_b1) annotation (Line(points={{-18,-58},
          {-18,-22},{-16,-22}}, color={0,127,255}));
  connect(junCwPumSup.port_2, res1.port_a)
    annotation (Line(points={{-8,-68},{-8,-86}}, color={0,127,255}));
  connect(gai.u, gai3.u) annotation (Line(points={{176,-138},{178,-138},{178,-188},
          {-126,-188},{-126,-84}}, color={0,0,127}));
  connect(junChwPumRet.port_2, Pump_CHW_Primary1.port_a) annotation (Line(
        points={{70,-104},{64,-104},{64,-118},{54,-118}}, color={0,127,255}));
  connect(junChwPumRet.port_3, Pump_CHW_Primary2.port_a) annotation (Line(
        points={{80,-94},{65,-94},{65,-72},{54,-72}}, color={0,127,255}));
  connect(Pump_CHW_Primary2.port_b, chi2.port_a2) annotation (Line(points={{34,
          -72},{28,-72},{28,-70},{-4,-70},{-4,-22}}, color={0,127,255}));
  connect(Pump_CHW_Primary1.port_b, chi1.port_a2) annotation (Line(points={{34,
          -118},{28,-118},{28,-22}}, color={0,127,255}));
  connect(chi1.port_b2, junChwPumSup.port_1) annotation (Line(points={{28,-2},{
          34,-2},{34,20},{8,20},{8,78},{14,78}}, color={0,127,255}));
  connect(junChwPumSup.port_3, chi2.port_b2) annotation (Line(points={{24,68},{
          24,36},{-4,36},{-4,-2}}, color={0,127,255}));
  connect(junChwPumSup.port_2, Sensor_TCHW_primary_supply.port) annotation (
      Line(points={{34,78},{52,78},{52,76},{62,76},{62,98}}, color={0,127,255}));
  connect(junSecRet.port_2, Sensor_TCHW_return.port) annotation (Line(points={{
          328,-82},{328,-84},{296,-84},{296,-116},{278,-116},{278,-108}}, color
        ={0,127,255}));
  connect(Sensor_mCHi.port_b, junChwPumRet.port_1) annotation (Line(points={{
          104,-118},{104,-116},{90,-116},{90,-104}}, color={0,127,255}));
  connect(junChwPumSup.port_2, junPriSup.port_1) annotation (Line(points={{34,
          78},{52,78},{52,76},{84,76},{84,70},{90,70}}, color={0,127,255}));
  connect(junPriSup.port_2, junSecSup.port_1) annotation (Line(points={{110,70},
          {124,70},{124,28},{168,28}}, color={0,127,255}));
  connect(junSecRet.port_2, junPriRet.port_1) annotation (Line(points={{328,-82},
          {328,-84},{296,-84},{296,-116},{180,-116},{180,-92},{172,-92}}, color
        ={0,127,255}));
  connect(junPriRet.port_2, Sensor_mCHi.port_a) annotation (Line(points={{152,
          -92},{132,-92},{132,-118},{124,-118}}, color={0,127,255}));
  connect(junPriSup.port_3, Pump_CHW_Bypass.port_a) annotation (Line(points={{
          100,60},{100,-36},{110,-36},{110,-44}}, color={0,127,255}));
  connect(Pump_CHW_Bypass.port_b, junPriRet.port_3) annotation (Line(points={{
          110,-64},{110,-72},{162,-72},{162,-82}}, color={0,127,255}));
  connect(chiller_tes_plant_controller.chiller1On, chi1.on) annotation (Line(
        points={{192,200},{202,200},{202,208},{44,208},{44,0},{19,0}}, color={255,
          0,255}));
  connect(chiller_tes_plant_controller.chiller2On, chi2.on) annotation (Line(
        points={{192,195.2},{192,-14},{-13,-14},{-13,0}}, color={255,0,255}));
  connect(gai2.y, Pump_CHW_Bypass.m_flow_in)
    annotation (Line(points={{140,-54},{122,-54}}, color={0,0,127}));
  connect(booToRea.y, gai2.u)
    annotation (Line(points={{186,-58},{188,-54},{164,-54}}, color={0,0,127}));
  connect(chiller_tes_plant_controller.bypassPumpOn, booToRea.u) annotation (
      Line(points={{192,185.8},{200,185.8},{200,176},{210,176},{210,-58}},
        color={255,0,255}));
  connect(loaVar.y, chiller_tes_plant_controller.zoneThermalLoad) annotation (
      Line(points={{689,-90},{689,128},{688,128},{688,216},{150,216},{150,186.2},
          {168.2,186.2}}, color={0,0,127}));
  connect(chiller_tes_plant_controller.chiller1On, swi.u2) annotation (Line(
        points={{192,200},{202,200},{202,202},{212,202},{212,-150},{224,-150}},
        color={255,0,255}));
  connect(con.y, swi.u3) annotation (Line(points={{216,-182},{216,-172},{224,-172},
          {224,-158}}, color={0,0,127}));
  connect(chiller_tes_plant_controller.pumpFlowFraction, swi.u1) annotation (
      Line(points={{180.2,182},{180,182},{180,156},{196,156},{196,-142},{224,-142}},
        color={0,0,127}));
  connect(swi.y, gai.u) annotation (Line(points={{248,-150},{256,-150},{256,-152},
          {260,-152},{260,-176},{186,-176},{186,-138},{176,-138}}, color={0,0,127}));
  connect(gai.y, Pump_CHW_Primary1.m_flow_in) annotation (Line(points={{152,-138},
          {44,-138},{44,-130}}, color={0,0,127}));
  connect(gai3.y, pumCW1.m_flow_in) annotation (Line(points={{-126,-61},{-124,-61},
          {-124,12},{-116,12}}, color={0,0,127}));
  connect(booToRea1.u, chiller_tes_plant_controller.chiller2On) annotation (
      Line(points={{74,-38},{84,-38},{84,-30},{206,-30},{206,195.2},{192,195.2}},
        color={255,0,255}));
  connect(gai4.y, Pump_CHW_Primary2.m_flow_in)
    annotation (Line(points={{66,-82},{68,-84},{44,-84}}, color={0,0,127}));
  connect(booToRea1.y, gai4.u) annotation (Line(points={{50,-38},{48,-38},{48,-50},
          {90,-50},{90,-82}}, color={0,0,127}));
  connect(pumCW2.m_flow_in, gai5.y) annotation (Line(points={{-112,-38},{-114,-38},
          {-114,-57},{-88,-57}}, color={0,0,127}));
  connect(booToRea1.y, gai5.u) annotation (Line(points={{50,-38},{48,-38},{48,-56},
          {12,-56},{12,-80},{-72,-80},{-72,-88},{-88,-88},{-88,-80}}, color={0,0,
          127}));
  connect(gai1.y, Pump_CHW_Secondary.m_flow_in)
    annotation (Line(points={{344,70},{350,70},{350,44}}, color={0,0,127}));
  connect(chiller_tes_plant_controller.secondaryPumpOn, swi1.u2) annotation (
      Line(points={{192,191.2},{224,191.2},{224,188},{254,188},{254,78},{266,78}},
        color={255,0,255}));
  connect(con1.y, swi1.u3) annotation (Line(points={{274,46},{276,46},{276,56},{
          266,56},{266,70}}, color={0,0,127}));
  connect(swi1.y, gai1.u) annotation (Line(points={{290,78},{308,78},{308,70},{320,
          70}}, color={0,0,127}));
  connect(chiller_tes_plant_controller.pumpFlowFraction, swi1.u1) annotation (
      Line(points={{180.2,182},{182,182},{182,86},{266,86}}, color={0,0,127}));
  connect(Sensor_mCHi.port_b, Sensor_TCHW_primary_return.port) annotation (Line(
        points={{104,-118},{104,-184},{82,-184},{82,-176}}, color={0,127,255}));
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
      StopTime=345600,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end Chiller_Storage_CoolingTower_ParallelChillerWithCustomControl1;
