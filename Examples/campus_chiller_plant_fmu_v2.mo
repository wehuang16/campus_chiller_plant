within campus_chiller_plant.Examples;
model campus_chiller_plant_fmu_v2 "Put another chiller"
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
        rotation=180,
        origin={-222,-128})));
  parameter
    Buildings.Fluid.Chillers.Data.ElectricEIR.ElectricEIRChiller_McQuay_WSC_471kW_5_89COP_Vanes
    per1(
    QEva_flow_nominal(displayUnit="kW") = -2989000,
    COP_nominal=7,
    PLRMax=1,
    PLRMinUnl=0.4,
    mEva_flow_nominal=62.79*0.9997,
    mCon_flow_nominal=62.79*1.5*0.9997,
    TEvaLvg_nominal=278.43,
    capFunT={0.70790824,-0.002006568,-0.00259605,0.030058776,-0.0010564344,0.0020457036},
    EIRFunT={0.5605438,-0.01377927,6.57072e-005,0.013219362,0.000268596,-0.0005011308},
    EIRFunPLR={0.17149273,0.58820208,0.23737257},
    TEvaLvgMin=278.43,
    TEvaLvgMax=283.71) "Chiller performance data"
    annotation (Placement(transformation(extent={{-294,152},{-252,194}})));

  Buildings.Fluid.Chillers.ElectricEIR chi1(
    redeclare package Medium1 = CondensorWater,
    redeclare package Medium2 = ChilledWater,
    per=per1,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    dp1_nominal=dP0,
    dp2_nominal=dP0) "Chiller model" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-50,-12})));
  Buildings.Fluid.Movers.FlowControlled_m_flow Pump_CHW_Primary1(
    redeclare package Medium = ChilledWater,
    addPowerToMedium=false,
    m_flow_nominal=mEva_flow_nominal,
    tau=30,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    "Pump for chilled water loop" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-44,-82})));
  Modelica.Blocks.Sources.Constant SP_TCHe[2](k={273.15 + 5.28,273.15 + 4.17})
    "chilled water suppy temperature setpoint"
                              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-122,160})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_mS(redeclare package Medium =
        ChilledWater)                                                                   annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={262,52})));
  Buildings.Fluid.Movers.FlowControlled_m_flow Pump_CHW_Secondary(
    redeclare package Medium = ChilledWater,
    addPowerToMedium=false,
    m_flow_nominal=mEva_flow_nominal,
    tau=30,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    "Pump for chilled water loop" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={350,88})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_msup(redeclare package Medium =
        ChilledWater)                                                                     annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={434,88})));
  Buildings.Fluid.FixedResistances.PressureDrop res2(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    dp_nominal=dP0)                                                                         annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={478,88})));
  Buildings.Fluid.FixedResistances.Junction junSecSup(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={mEva_flow_nominal,-0.9*mEva_flow_nominal,-0.1*
        mEva_flow_nominal},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={262,90})));
  Buildings.Fluid.FixedResistances.Junction junSecRet(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={0.9*mEva_flow_nominal,-mEva_flow_nominal,0.1*
        mEva_flow_nominal},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={264,-130})));
  Buildings.Fluid.FixedResistances.PressureDrop res3(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    dp_nominal=dP0)                                                                         annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={264,-82})));
  Buildings.Fluid.Chillers.ElectricEIR chi2(
    redeclare package Medium1 = CondensorWater,
    redeclare package Medium2 = ChilledWater,
    per=per2,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    dp1_nominal=dP0,
    dp2_nominal=dP0) "Chiller model" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-86,-12})));
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
        origin={-319,14})));
  Buildings.Fluid.Movers.FlowControlled_m_flow pumCW1(
    redeclare package Medium = CondensorWater,
    addPowerToMedium=false,
    m_flow_nominal=mCon_flow_nominal,
    dp(start=dP0),
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    "Condenser water pump" annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={-160,44})));
  Buildings.Fluid.Storage.ExpansionVessel expVesChi(redeclare package Medium =
        CondensorWater, V_start=1)
    annotation (Placement(transformation(extent={{-256,-105},{-236,-85}})));
  Modelica.Blocks.Sources.Constant Twb(k=273.15 + 10) "chilled water flow rate"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-424,26})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_mCW(redeclare package Medium =
        CondensorWater)                                                                 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-252,16})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCWS(redeclare package Medium =
        CondensorWater)
    annotation (Placement(transformation(extent={{-246,42},{-226,62}})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCWR(redeclare package Medium =
        CondensorWater)
    annotation (Placement(transformation(extent={{-292,-116},{-272,-96}})));
  Modelica.Blocks.Math.Gain gai3(k=mCon_flow_nominal) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-204,-72})));
  Modelica.Blocks.Continuous.LimPID PID(yMax=1, yMin=0)
    annotation (Placement(transformation(extent={{-414,56},{-394,76}})));
  Modelica.Blocks.Sources.Constant SP_TCWS(k=273.15 + 10)
                              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-464,46})));
  Buildings.Fluid.Sources.Boundary_pT bou(redeclare package Medium =
        ChilledWater, nPorts=1)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={500,-42})));
  Buildings.Fluid.Storage.StratifiedEnhanced
                                     tan(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    VTan=2650,
    hTan=8.4352,
    dIns=0.01,
    nSeg=16,
    T_start=281.65)
    annotation (Placement(transformation(extent={{-13,-13},{13,13}},
        rotation=180,
        origin={263,-21})));
  BaseClasses.chiller_tes_plant_controller_ParallelChillerWithCustomControl chiller_tes_plant_controller
    annotation (Placement(transformation(extent={{170,184},{190,204}})));
  BaseClasses.TesStatusController
                      tesStatusController(tempDischargedBottom(displayUnit=
          "degC") = 285.93, tempChargedTop(displayUnit="degC") = 278.71)
    annotation (Placement(transformation(extent={{260,190},{280,210}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis hys(uLow=0.05, uHigh=0.1)
    annotation (Placement(transformation(extent={{660,116},{680,136}})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai(k=mEva_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={166,-164})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai1(k=mEva_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={430,170})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow fixHeaFlo(T_ref=
        293.15)
    "Fixed heat flow rate"
    annotation (Placement(transformation(extent={{808,36},{828,56}})));
  Buildings.Fluid.MixingVolumes.MixingVolume vol(
    T_start=283.71,
    nPorts=3,
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    V=200,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Mixing volume"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={536,-42})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=vol.heatPort.T)
    annotation (Placement(transformation(extent={{676,-68},{696,-48}})));
  Buildings.Fluid.Movers.FlowControlled_m_flow pumCW2(
    redeclare package Medium = CondensorWater,
    addPowerToMedium=false,
    m_flow_nominal=mCon_flow_nominal,
    dp(start=dP0),
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    "Condenser water pump" annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={-156,-6})));
  Buildings.Fluid.FixedResistances.Junction junCwPumSup(
    redeclare package Medium = CondensorWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-184,-128})));
  Buildings.Fluid.FixedResistances.Junction junCwPumRet(
    redeclare package Medium = CondensorWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-216,16})));
  Buildings.Fluid.FixedResistances.Junction junChwPumRet(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-10,-130})));
  Buildings.Fluid.Movers.FlowControlled_m_flow Pump_CHW_Primary2(
    redeclare package Medium = ChilledWater,
    addPowerToMedium=false,
    m_flow_nominal=mEva_flow_nominal,
    tau=30,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    "Pump for chilled water loop" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-80,-82})));
  Buildings.Fluid.FixedResistances.Junction junChwPumSup(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={22,90})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_mCHi(redeclare package Medium =
        ChilledWater)                                                                     annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={66,-130})));
  Buildings.Fluid.Movers.FlowControlled_m_flow Pump_CHW_Bypass(
    redeclare package Medium = ChilledWater,
    addPowerToMedium=false,
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
        origin={110,90})));
  Buildings.Fluid.FixedResistances.Junction junPriRet(
    redeclare package Medium = ChilledWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={110,-130})));
  parameter
    Buildings.Fluid.Chillers.Data.ElectricEIR.ElectricEIRChiller_McQuay_WSC_471kW_5_89COP_Vanes
    per2(
    QEva_flow_nominal(displayUnit="kW") = -3165000,
    COP_nominal=7,
    PLRMax=1,
    PLRMinUnl=0.4,
    mEva_flow_nominal=62.79*0.9997,
    mCon_flow_nominal=62.79*1.5*0.9997,
    TEvaLvg_nominal=277.32,
    capFunT={0.70790824,-0.002006568,-0.00259605,0.030058776,-0.0010564344,0.0020457036},
    EIRFunT={0.5605438,-0.01377927,6.57072e-005,0.013219362,0.000268596,-0.0005011308},
    EIRFunPLR={0.17149273,0.58820208,0.23737257},
    TEvaLvgMin=277.32,
    TEvaLvgMax=283.71) "Chiller performance data"
    annotation (Placement(transformation(extent={{-236,152},{-194,194}})));

  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai2(k=25.08*0.9997)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={152,-54})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={192,-54})));
  Buildings.Controls.OBC.CDL.Reals.Switch swi
    annotation (Placement(transformation(extent={{228,-212},{248,-192}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con(k=0)
    annotation (Placement(transformation(extent={{194,-218},{214,-198}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea1(realTrue=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-58,-222})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai4(k=57.08*0.9997)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-56,-174})));
  Modelica.Blocks.Math.Gain gai5(k=mCon_flow_nominal) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-146,-68})));
  Buildings.Controls.OBC.CDL.Reals.Switch swi1
    annotation (Placement(transformation(extent={{412,206},{432,226}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con1(k=0)
    annotation (Placement(transformation(extent={{372,164},{392,184}})));
  Buildings.Controls.OBC.CDL.Reals.Line lin
    annotation (Placement(transformation(extent={{778,134},{798,154}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con2(k=273.15 + 11.11)
    annotation (Placement(transformation(extent={{512,230},{532,250}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con3(k=1)
    annotation (Placement(transformation(extent={{724,164},{744,184}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con4(k=273.15 + 11.67)
    annotation (Placement(transformation(extent={{742,116},{762,136}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con5(k=0)
    annotation (Placement(transformation(extent={{748,74},{768,94}})));
  Buildings.Controls.OBC.CDL.Reals.Multiply loaAct annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={874,-74})));
  Buildings.Controls.Continuous.LimPID conPID(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.3,
    Ti=150,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    y_start=0.2,
    reverseActing=false)
    annotation (Placement(transformation(extent={{752,0},{772,20}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con6(k=273.15 + 10.56)
    annotation (Placement(transformation(extent={{704,-12},{722,6}})));

  Buildings.Controls.OBC.CDL.Reals.Hysteresis OAT_lockout(uLow=19.95, uHigh=20)
    annotation (Placement(transformation(extent={{620,214},{640,234}})));
  Buildings.Controls.OBC.CDL.Logical.And and2
    annotation (Placement(transformation(extent={{444,270},{464,290}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor tempTan[16]
    " tank tempearture" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={358,-310})));
  Buildings.Controls.OBC.CDL.Reals.MultiSum mulSum(nin=16)
    annotation (Placement(transformation(extent={{402,-320},{422,-300}})));
  Modelica.Blocks.Math.Gain gai6(k=1/16) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={468,-310})));
  Buildings.Controls.OBC.UnitConversions.To_degC tank_average_temperature_simulation
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={520,-310})));

  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_TCHW_supply_1(redeclare
      package Medium = ChilledWater, m_flow_nominal=mEva_flow_nominal,
    T_start=278.43)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-44,68})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_TCHW_supply_2(redeclare
      package Medium = ChilledWater, m_flow_nominal=mEva_flow_nominal,
    T_start=277.32)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-80,68})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_TCHW_primary_supply(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    T_start=278.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={64,90})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_TCHW_primary_return(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    T_start=283.71) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={30,-130})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_Ttankout(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    T_start=277.32)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={262,16})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_Ttankin(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    T_start=283.71)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={264,-52})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_TCHWR(redeclare package
      Medium = ChilledWater, m_flow_nominal=mEva_flow_nominal) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={468,-130})));
  Buildings.HeatTransfer.Sources.PrescribedHeatFlow heat_gain_top
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={358,-368})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con7(k=305.01)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={400,-368})));
  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput systemCommand
    annotation (Placement(transformation(extent={{-546,224},{-506,264}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput outdoor_air_temperature
    annotation (Placement(transformation(extent={{-554,166},{-514,206}})));
  Buildings.Controls.OBC.UnitConversions.To_degC to_degC
    annotation (Placement(transformation(extent={{-392,186},{-372,206}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput building_cooling_load
    annotation (Placement(transformation(extent={{-534,88},{-494,128}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_TCHWS(redeclare package
      Medium = ChilledWater, m_flow_nominal=mEva_flow_nominal) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={312,90})));
  BaseClasses.FmuPatch fmuPatch
    annotation (Placement(transformation(extent={{-442,-164},{-422,-144}})));
equation
  connect(Pump_CHW_Secondary.port_b, Sensor_msup.port_a)
    annotation (Line(points={{360,88},{424,88}}, color={0,127,255}));
  connect(Sensor_msup.port_b, res2.port_a)
    annotation (Line(points={{444,88},{468,88}},         color={0,127,255}));
  connect(junSecSup.port_3, Sensor_mS.port_a)
    annotation (Line(points={{262,80},{262,62}},           color={0,127,255}));
  connect(res3.port_b, junSecRet.port_3) annotation (Line(points={{264,-92},{
          264,-120}},          color={0,127,255}));
  connect(SP_TCHe[2].y, chi2.TSet)
    annotation (Line(points={{-111,160},{-100,160},{-100,8},{-83,8},{-83,0}},
                                                       color={0,0,127}));
  connect(SP_TCHe[1].y, chi1.TSet)
    annotation (Line(points={{-111,160},{-100,160},{-100,8},{-47,8},{-47,0}},
                                                         color={0,0,127}));
  connect(res1.port_b, expVesChi.port_a) annotation (Line(points={{-232,-128},{
          -246,-128},{-246,-105}},
                                 color={0,127,255}));
  connect(Twb.y, cooTow.TAir)
    annotation (Line(points={{-413,26},{-353.8,26}}, color={0,0,127}));
  connect(expVesChi.port_a, cooTow.port_a) annotation (Line(points={{-246,-105},
          {-246,-128},{-366,-128},{-366,14},{-348,14}},             color={0,
          127,255}));
  connect(Sensor_mCW.port_b, Sensor_TCWS.port)
    annotation (Line(points={{-242,16},{-236,16},{-236,42}},
                                                 color={0,127,255}));
  connect(expVesChi.port_a, Sensor_TCWR.port) annotation (Line(points={{-246,
          -105},{-246,-128},{-282,-128},{-282,-116}},
                                                    color={0,127,255}));
  connect(PID.y, cooTow.y) annotation (Line(points={{-393,66},{-380,66},{-380,
          38},{-353.8,38}}, color={0,0,127}));
  connect(SP_TCWS.y, PID.u_m)
    annotation (Line(points={{-453,46},{-404,46},{-404,54}}, color={0,0,127}));
  connect(Sensor_TCWS.T, PID.u_s) annotation (Line(points={{-229,52},{-214,52},
          {-214,88},{-430,88},{-430,66},{-416,66}},
                                                  color={0,0,127}));
  connect(tesStatusController.TesMode, chiller_tes_plant_controller.tesStatus)
    annotation (Line(points={{282,200},{288,200},{288,220},{156,220},{156,191.6},
          {167.8,191.6}}, color={255,127,0}));
  connect(fixHeaFlo.port,vol.heatPort)
    annotation (Line(points={{828,46},{836,46},{836,28},{620,28},{620,-24},{536,
          -24},{536,-32}},                                    color={191,0,0}));
  connect(cooTow.port_b, Sensor_mCW.port_a) annotation (Line(points={{-290,14},
          {-290,16},{-262,16}}, color={0,127,255}));
  connect(Sensor_mCW.port_b, junCwPumRet.port_1) annotation (Line(points={{-242,16},
          {-226,16}},                       color={0,127,255}));
  connect(junCwPumRet.port_2, pumCW1.port_a) annotation (Line(points={{-206,16},
          {-192,16},{-192,44},{-170,44}},
                                        color={0,127,255}));
  connect(junCwPumRet.port_3, pumCW2.port_a) annotation (Line(points={{-216,6},
          {-216,0},{-176,0},{-176,-6},{-166,-6}},
                                  color={0,127,255}));
  connect(pumCW1.port_b, chi1.port_a1)
    annotation (Line(points={{-150,44},{-56,44},{-56,-2}},
                                                         color={0,127,255}));
  connect(pumCW2.port_b, chi2.port_a1) annotation (Line(points={{-146,-6},{-104,
          -6},{-104,-4},{-92,-4},{-92,-2}},   color={0,127,255}));
  connect(junCwPumSup.port_1, chi1.port_b1) annotation (Line(points={{-174,-128},
          {-124,-128},{-124,-68},{-56,-68},{-56,-22}},
                                      color={0,127,255}));
  connect(junCwPumSup.port_3, chi2.port_b1) annotation (Line(points={{-184,-118},
          {-184,-40},{-92,-40},{-92,-22}},
                                color={0,127,255}));
  connect(junCwPumSup.port_2, res1.port_a)
    annotation (Line(points={{-194,-128},{-212,-128}},
                                                 color={0,127,255}));
  connect(gai.u, gai3.u) annotation (Line(points={{178,-164},{178,-188},{-204,
          -188},{-204,-84}},       color={0,0,127}));
  connect(junPriSup.port_2, junSecSup.port_1) annotation (Line(points={{120,90},
          {252,90}},                   color={0,127,255}));
  connect(junSecRet.port_2, junPriRet.port_1) annotation (Line(points={{254,
          -130},{120,-130}},                                              color
        ={0,127,255}));
  connect(junPriRet.port_2, Sensor_mCHi.port_a) annotation (Line(points={{100,
          -130},{76,-130}},                      color={0,127,255}));
  connect(junPriSup.port_3, Pump_CHW_Bypass.port_a) annotation (Line(points={{110,80},
          {110,-44}},                             color={0,127,255}));
  connect(Pump_CHW_Bypass.port_b, junPriRet.port_3) annotation (Line(points={{110,-64},
          {110,-120}},                             color={0,127,255}));
  connect(chiller_tes_plant_controller.chiller1On, chi1.on) annotation (Line(
        points={{192,200},{202,200},{202,208},{44,208},{44,0},{-53,0}},color={255,
          0,255}));
  connect(chiller_tes_plant_controller.chiller2On, chi2.on) annotation (Line(
        points={{192,195.2},{192,-14},{-89,-14},{-89,0}}, color={255,0,255}));
  connect(gai2.y, Pump_CHW_Bypass.m_flow_in)
    annotation (Line(points={{140,-54},{122,-54}}, color={0,0,127}));
  connect(booToRea.y, gai2.u)
    annotation (Line(points={{180,-54},{164,-54}},           color={0,0,127}));
  connect(chiller_tes_plant_controller.bypassPumpOn, booToRea.u) annotation (
      Line(points={{192,185.8},{200,185.8},{200,176},{204,176},{204,-54}},
        color={255,0,255}));
  connect(chiller_tes_plant_controller.chiller1On, swi.u2) annotation (Line(
        points={{192,200},{292,200},{292,180},{280,180},{280,92},{232,92},{232,
          -4},{280,-4},{280,-36},{300,-36},{300,-80},{396,-80},{396,-204},{226,
          -204},{226,-202}},
        color={255,0,255}));
  connect(con.y, swi.u3) annotation (Line(points={{216,-208},{216,-210},{226,
          -210}},      color={0,0,127}));
  connect(gai.y, Pump_CHW_Primary1.m_flow_in) annotation (Line(points={{154,
          -164},{-56,-164},{-56,-82}},
                                color={0,0,127}));
  connect(gai3.y, pumCW1.m_flow_in) annotation (Line(points={{-204,-61},{-204,
          -8},{-188,-8},{-188,20},{-160,20},{-160,32}},
                                color={0,0,127}));
  connect(booToRea1.u, chiller_tes_plant_controller.chiller2On) annotation (
      Line(points={{-58,-234},{84,-234},{84,-30},{206,-30},{206,195.2},{192,
          195.2}},
        color={255,0,255}));
  connect(pumCW2.m_flow_in, gai5.y) annotation (Line(points={{-156,-18},{-156,
          -48},{-146,-48},{-146,-57}},
                                 color={0,0,127}));
  connect(gai1.y, Pump_CHW_Secondary.m_flow_in)
    annotation (Line(points={{442,170},{448,170},{448,112},{350,112},{350,100}},
                                                          color={0,0,127}));
  connect(chiller_tes_plant_controller.secondaryPumpOn, swi1.u2) annotation (
      Line(points={{192,191.2},{252,191.2},{252,180},{276,180},{276,176},{332,
          176},{332,216},{410,216}},
        color={255,0,255}));
  connect(con1.y, swi1.u3) annotation (Line(points={{394,174},{410,174},{410,
          208}},             color={0,0,127}));
  connect(realExpression.y, lin.u) annotation (Line(points={{697,-58},{484,-58},
          {484,146},{776,146},{776,144}}, color={0,0,127}));
  connect(con2.y, lin.x1) annotation (Line(points={{534,240},{548,240},{548,152},
          {776,152}}, color={0,0,127}));
  connect(con3.y, lin.f1) annotation (Line(points={{746,174},{746,144},{776,144},
          {776,148}},           color={0,0,127}));
  connect(con4.y, lin.x2) annotation (Line(points={{764,126},{764,140},{776,140}},
                      color={0,0,127}));
  connect(con5.y, lin.f2)
    annotation (Line(points={{770,84},{776,84},{776,136}}, color={0,0,127}));
  connect(lin.y, loaAct.u1) annotation (Line(points={{800,144},{800,136},{862,
          136},{862,-68}},           color={0,0,127}));
  connect(realExpression.y, conPID.u_m) annotation (Line(points={{697,-58},{697,
          -60},{762,-60},{762,-2}},                      color={0,0,127}));
  connect(con6.y, conPID.u_s) annotation (Line(points={{723.8,-3},{740,-3},{740,
          10},{750,10}},      color={0,0,127}));
  connect(conPID.y, hys.u) annotation (Line(points={{773,10},{780,10},{780,68},
          {648,68},{648,126},{658,126}},
                      color={0,0,127}));
  connect(swi1.y, gai1.u) annotation (Line(points={{434,216},{444,216},{444,188},
          {412,188},{412,170},{418,170}},
                    color={0,0,127}));
  connect(conPID.y, swi1.u1) annotation (Line(points={{773,10},{780,10},{780,68},
          {648,68},{648,136},{452,136},{452,200},{404,200},{404,224},{410,224}},
                                             color={0,0,127}));
  connect(swi.y, gai.u) annotation (Line(points={{250,-202},{272,-202},{272,
          -164},{178,-164}}, color={0,0,127}));
  connect(conPID.y, swi.u1) annotation (Line(points={{773,10},{780,10},{780,68},
          {648,68},{648,136},{452,136},{452,-232},{188,-232},{188,-184},{226,
          -184},{226,-194}},            color={0,0,127}));
  connect(OAT_lockout.y, and2.u1) annotation (Line(points={{642,224},{658,224},
          {658,234},{668,234},{668,324},{442,324},{442,280}}, color={255,0,255}));
  connect(hys.y, and2.u2) annotation (Line(points={{682,126},{682,264},{442,264},
          {442,272}},           color={255,0,255}));
  connect(mulSum.y, gai6.u) annotation (Line(points={{424,-310},{456,-310}},
                             color={0,0,127}));
  connect(tempTan.T,mulSum. u)
    annotation (Line(points={{369,-310},{400,-310}},
                                                color={0,0,127}));
  connect(tan.heaPorVol,tempTan. port) annotation (Line(points={{263,-21},{328,
          -21},{328,-310},{348,-310}},       color={191,0,0}));
  connect(tempTan[3].T, tesStatusController.TesTopTemp) annotation (Line(points={{369,
          -310},{369,-308},{376,-308},{376,-40},{368,-40},{368,116},{252,116},{
          252,176},{248,176},{248,184},{296,184},{296,224},{252,224},{252,212},
          {248,212},{248,203.8},{257.8,203.8}},
                         color={0,0,127}));
  connect(tempTan[14].T, tesStatusController.TesBottomTemp) annotation (Line(
        points={{369,-310},{369,-308},{376,-308},{376,-40},{368,-40},{368,116},
          {252,116},{252,176},{248,176},{248,184},{258,184},{258,193.8}}, color
        ={0,0,127}));
  connect(gai6.y, tank_average_temperature_simulation.u) annotation (Line(
        points={{479,-310},{508,-310}},                       color={0,0,127}));
  connect(Sensor_TCHW_supply_1.port_a, chi1.port_b2) annotation (Line(points={{-44,58},
          {-44,-2}},
                color={0,127,255}));
  connect(Sensor_mCHi.port_b, Sensor_TCHW_primary_return.port_a) annotation (
      Line(points={{56,-130},{40,-130}},             color={0,127,255}));
  connect(junChwPumRet.port_1, Sensor_TCHW_primary_return.port_b) annotation (
      Line(points={{0,-130},{20,-130}},                                color={0,
          127,255}));
  connect(Sensor_mS.port_b, Sensor_Ttankout.port_a) annotation (Line(points={{262,42},
          {262,26}},                                         color={0,127,255}));
  connect(Sensor_Ttankout.port_b, tan.port_b) annotation (Line(points={{262,6},
          {263,6},{263,-8}},    color={0,127,255}));
  connect(junChwPumSup.port_2, Sensor_TCHW_primary_supply.port_a)
    annotation (Line(points={{32,90},{54,90}}, color={0,127,255}));
  connect(Sensor_TCHW_primary_supply.port_b, junPriSup.port_1) annotation (Line(
        points={{74,90},{100,90}},                color={0,127,255}));
  connect(Sensor_TCHWR.port_b, junSecRet.port_1) annotation (Line(points={{458,
          -130},{274,-130}},                   color={0,127,255}));
  connect(res3.port_a, Sensor_Ttankin.port_b)
    annotation (Line(points={{264,-72},{264,-62}},         color={0,127,255}));
  connect(Sensor_Ttankin.port_a, tan.port_a)
    annotation (Line(points={{264,-42},{263,-42},{263,-34}},
                                                           color={0,127,255}));
  connect(heat_gain_top.port, tan.heaPorVol[1]) annotation (Line(points={{348,
          -368},{328,-368},{328,-20.6344},{263,-20.6344}},
                      color={191,0,0}));
  connect(con7.y, heat_gain_top.Q_flow)
    annotation (Line(points={{388,-368},{368,-368}},
                                                   color={0,0,127}));
  connect(and2.y, chiller_tes_plant_controller.loadRequest) annotation (Line(
        points={{466,280},{490,280},{490,232},{160,232},{160,197.8},{168,197.8}},
        color={255,0,255}));
  connect(outdoor_air_temperature, to_degC.u) annotation (Line(points={{-534,
          186},{-404,186},{-404,196},{-394,196}}, color={0,0,127}));
  connect(to_degC.y, OAT_lockout.u) annotation (Line(points={{-370,196},{618,
          196},{618,224}}, color={0,0,127}));
  connect(building_cooling_load, loaAct.u2) annotation (Line(points={{-514,108},
          {810,108},{810,-80},{862,-80}}, color={0,0,127}));
  connect(systemCommand, chiller_tes_plant_controller.systemCommand)
    annotation (Line(points={{-526,244},{164,244},{164,201.8},{168,201.8}},
        color={255,127,0}));
  connect(junSecSup.port_2, Sensor_TCHWS.port_a) annotation (Line(points={{272,90},
          {302,90}},                       color={0,127,255}));
  connect(Sensor_TCHWS.port_b, Pump_CHW_Secondary.port_a)
    annotation (Line(points={{322,90},{340,90},{340,88}}, color={0,127,255}));
  connect(gai5.u, booToRea1.y) annotation (Line(points={{-146,-80},{-146,-210},
          {-58,-210}}, color={0,0,127}));
  connect(Pump_CHW_Primary2.m_flow_in, gai4.y) annotation (Line(points={{-92,
          -82},{-92,-156},{-56,-156},{-56,-162}}, color={0,0,127}));
  connect(gai4.u, booToRea1.y) annotation (Line(points={{-56,-186},{-56,-210},{
          -58,-210}}, color={0,0,127}));
  connect(Sensor_TCHW_supply_1.port_b, junChwPumSup.port_3) annotation (Line(
        points={{-44,78},{-44,88},{0,88},{0,72},{22,72},{22,80}}, color={0,127,
          255}));
  connect(Sensor_TCHW_supply_2.port_b, junChwPumSup.port_1)
    annotation (Line(points={{-80,78},{-80,90},{12,90}}, color={0,127,255}));
  connect(Sensor_TCHWR.port_a, vol.ports[1]) annotation (Line(points={{478,-130},
          {526,-130},{526,-40.6667}}, color={0,127,255}));
  connect(chi2.port_a2, Pump_CHW_Primary2.port_b)
    annotation (Line(points={{-80,-22},{-80,-72}}, color={0,127,255}));
  connect(chi1.port_a2, Pump_CHW_Primary1.port_b)
    annotation (Line(points={{-44,-22},{-44,-72}}, color={0,127,255}));
  connect(Sensor_TCHW_supply_2.port_a, chi2.port_b2)
    annotation (Line(points={{-80,58},{-80,-2}}, color={0,127,255}));
  connect(Pump_CHW_Primary1.port_a, junChwPumRet.port_3) annotation (Line(
        points={{-44,-92},{-44,-108},{-10,-108},{-10,-120}}, color={0,127,255}));
  connect(Pump_CHW_Primary2.port_a, junChwPumRet.port_2) annotation (Line(
        points={{-80,-92},{-80,-130},{-20,-130}}, color={0,127,255}));
  connect(res2.port_b, vol.ports[2]) annotation (Line(points={{488,88},{516,88},
          {516,-40},{520,-40},{520,-42},{526,-42}}, color={0,127,255}));
  connect(bou.ports[1], vol.ports[3]) annotation (Line(points={{510,-42},{512,
          -43.3333},{526,-43.3333}}, color={0,127,255}));
  connect(fixHeaFlo.Q_flow, loaAct.y) annotation (Line(points={{808,46},{808,
          -12},{886,-12},{886,-74}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-400,
            -160},{380,120}}),
                         graphics={Text(
          extent={{-84,38},{106,-92}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.None,
          textString="CHL"), Rectangle(extent={{-400,122},{380,-162}},
            lineColor={0,0,0})}),                                Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-400,-160},{380,
            120}})),
    experiment(
      StartTime=7171200,
      StopTime=8985600,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end campus_chiller_plant_fmu_v2;
