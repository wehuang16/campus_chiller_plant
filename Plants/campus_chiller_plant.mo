within campus_chiller_plant.Plants;
model campus_chiller_plant "Put another chiller"
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
    mEva_flow_nominal=65*0.9997,
    mCon_flow_nominal=65*1.5*0.9997,
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
    fraPFan_nominal=700000/0.94,
    PFan_nominal=700000,
    TAirInWB_nominal(displayUnit="degC") = 283.15,
    fanRelPow(r_P={0,0.1,0.3,0.6,1}),
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
  Modelica.Blocks.Sources.Constant Twb(k=273.15 + 10)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-424,26})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_mCW(redeclare package Medium =
        CondensorWater)                                                                 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-268,18})));
  Buildings.Controls.Continuous.LimPID
                                    conPID1(
    k=0.3,
    Ti=300,                             yMax=1, yMin=0,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    y_start=0,
    reverseActing=false)
    annotation (Placement(transformation(extent={{-414,56},{-394,76}})));
  Modelica.Blocks.Sources.Constant SP_TCWS(k=273.15 + 21.11)
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
  Subsequences.plant_mode_controller chiller_tes_plant_controller
    annotation (Placement(transformation(extent={{-354,260},{-334,280}})));
  Subsequences.TesStatusController tesStatusController(tempDischargedBottom(
        displayUnit="degC") = 285.93, tempChargedTop(displayUnit="degC") =
      278.71)
    annotation (Placement(transformation(extent={{-404,258},{-384,278}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow fixHeaFlo(T_ref=
        293.15)
    "Fixed heat flow rate"
    annotation (Placement(transformation(extent={{484,-4},{504,16}})));
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
        origin={-208,36})));
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
    mEva_flow_nominal=44*0.9997,
    mCon_flow_nominal=44*1.5*0.9997,
    TEvaLvg_nominal=277.32,
    capFunT={0.70790824,-0.002006568,-0.00259605,0.030058776,-0.0010564344,0.0020457036},
    EIRFunT={0.5605438,-0.01377927,6.57072e-005,0.013219362,0.000268596,-0.0005011308},
    EIRFunPLR={0.17149273,0.58820208,0.23737257},
    TEvaLvgMin=277.32,
    TEvaLvgMax=283.71) "Chiller performance data"
    annotation (Placement(transformation(extent={{-236,152},{-194,194}})));

  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con6(k=273.15 + 10.56)
    annotation (Placement(transformation(extent={{562,0},{580,18}})));

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
  Buildings.Controls.OBC.CDL.Interfaces.RealInput outside_air_temperature
    annotation (Placement(transformation(extent={{-554,166},{-514,206}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput building_cooling_load
    annotation (Placement(transformation(extent={{-534,88},{-494,128}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_TCHWS(redeclare package
      Medium = ChilledWater, m_flow_nominal=mEva_flow_nominal) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={312,90})));
  Subsequences.FmuPatch fmuPatch
    annotation (Placement(transformation(extent={{-442,-164},{-422,-144}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_TCWR(redeclare package
      Medium = CondensorWater, m_flow_nominal=mCon_flow_nominal) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-326,-128})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_TCWS(redeclare package
      Medium = CondensorWater, m_flow_nominal=mCon_flow_nominal) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-248,56})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput building_cooling_load_actually_served
    annotation (Placement(transformation(extent={{1040,42},{1080,82}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput chiller_1_OnOff
    annotation (Placement(transformation(extent={{1046,-32},{1086,8}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput chiller_1_supply_water_temperature
    annotation (Placement(transformation(extent={{1054,-112},{1094,-72}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput chiller_1_flow_rate
    annotation (Placement(transformation(extent={{1046,-184},{1086,-144}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput chiller_1_thermal_power
    annotation (Placement(transformation(extent={{1056,-258},{1096,-218}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput chiller_1_electrical_power
    annotation (Placement(transformation(extent={{1056,-332},{1096,-292}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput chiller_2_OnOff
    annotation (Placement(transformation(extent={{1314,-24},{1354,16}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput chiller_2_supply_water_temperature
    annotation (Placement(transformation(extent={{1326,-88},{1366,-48}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput chiller_2_flow_rate
    annotation (Placement(transformation(extent={{1334,-168},{1374,-128}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput chiller_2_thermal_power
    annotation (Placement(transformation(extent={{1344,-242},{1384,-202}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput chiller_2_electrical_power
    annotation (Placement(transformation(extent={{1344,-316},{1384,-276}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput chillers_supply_water_temperature
    annotation (Placement(transformation(extent={{1058,-398},{1098,-358}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput chillers_return_water_temperature
    annotation (Placement(transformation(extent={{1056,-466},{1096,-426}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput bypass_flow_rate
    annotation (Placement(transformation(extent={{1064,-512},{1104,-472}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput tank_flow_rate
    annotation (Placement(transformation(extent={{1068,-584},{1108,-544}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput tank_supply_water_temperature
    annotation (Placement(transformation(extent={{1058,-660},{1098,-620}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput tank_return_water_temperature
    annotation (Placement(transformation(extent={{1062,-724},{1102,-684}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput tank_average_temperature
    annotation (Placement(transformation(extent={{1360,-400},{1400,-360}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput tank_top_temperature
    annotation (Placement(transformation(extent={{1358,-456},{1398,-416}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput tank_layer_3_temperature
    annotation (Placement(transformation(extent={{1358,-528},{1398,-488}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput tank_layer_14_temperature
    annotation (Placement(transformation(extent={{1358,-644},{1398,-604}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput tank_layer_9_temperature
    annotation (Placement(transformation(extent={{1366,-598},{1406,-558}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput tank_bottom_temperature
    annotation (Placement(transformation(extent={{1366,-678},{1406,-638}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput building_cooling_load_request
    annotation (Placement(transformation(extent={{1364,-758},{1404,-718}})));
  Buildings.Controls.OBC.CDL.Interfaces.IntegerOutput tank_status
    annotation (Placement(transformation(extent={{1368,-820},{1408,-780}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=Sensor_Ttankin.T)
    annotation (Placement(transformation(extent={{944,-708},{964,-688}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=Sensor_Ttankout.T)
    annotation (Placement(transformation(extent={{948,-650},{968,-630}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=tan.port_a.m_flow)
    annotation (Placement(transformation(extent={{958,-586},{978,-566}})));
  Modelica.Blocks.Sources.RealExpression realExpression4(y=Pump_CHW_Bypass.port_a.m_flow)
    annotation (Placement(transformation(extent={{950,-518},{970,-498}})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=
        Sensor_TCHW_primary_return.T)
    annotation (Placement(transformation(extent={{956,-464},{976,-444}})));
  Modelica.Blocks.Sources.RealExpression realExpression6(y=
        Sensor_TCHW_primary_supply.T)
    annotation (Placement(transformation(extent={{956,-394},{976,-374}})));
  Modelica.Blocks.Sources.RealExpression realExpression7(y=chi1.P)
    annotation (Placement(transformation(extent={{960,-318},{980,-298}})));
  Modelica.Blocks.Sources.RealExpression realExpression8(y=chi1.Q2_flow)
    annotation (Placement(transformation(extent={{950,-254},{970,-234}})));
  Modelica.Blocks.Sources.RealExpression realExpression9(y=chi1.port_a2.m_flow)
    annotation (Placement(transformation(extent={{960,-168},{980,-148}})));
  Modelica.Blocks.Sources.RealExpression realExpression11(y=
        Sensor_TCHW_supply_1.T)
    annotation (Placement(transformation(extent={{952,-116},{972,-96}})));
  Modelica.Blocks.Sources.RealExpression realExpression12(y=fixHeaFlo.Q_flow)
    annotation (Placement(transformation(extent={{972,42},{992,62}})));
  Modelica.Blocks.Sources.RealExpression realExpression13(y=
        Sensor_TCHW_supply_2.T)
    annotation (Placement(transformation(extent={{1254,-86},{1274,-66}})));
  Modelica.Blocks.Sources.RealExpression realExpression14(y=chi2.port_a2.m_flow)
    annotation (Placement(transformation(extent={{1260,-142},{1280,-122}})));
  Modelica.Blocks.Sources.RealExpression realExpression15(y=chi2.Q2_flow)
    annotation (Placement(transformation(extent={{1262,-234},{1282,-214}})));
  Modelica.Blocks.Sources.RealExpression realExpression16(y=chi2.P)
    annotation (Placement(transformation(extent={{1260,-298},{1280,-278}})));
  Modelica.Blocks.Sources.RealExpression realExpression17(y=
        tank_average_temperature_simulation.y)
    annotation (Placement(transformation(extent={{1266,-392},{1286,-372}})));
  Modelica.Blocks.Sources.RealExpression realExpression18(y=tan.heaPorVol[1].T)
    annotation (Placement(transformation(extent={{1264,-454},{1284,-434}})));
  Modelica.Blocks.Sources.RealExpression realExpression19(y=tan.heaPorVol[3].T)
    annotation (Placement(transformation(extent={{1266,-522},{1286,-502}})));
  Modelica.Blocks.Sources.RealExpression realExpression20(y=tan.heaPorVol[9].T)
    annotation (Placement(transformation(extent={{1272,-592},{1292,-572}})));
  Modelica.Blocks.Sources.RealExpression realExpression21(y=tan.heaPorVol[14].T)
    annotation (Placement(transformation(extent={{1268,-632},{1288,-612}})));
  Modelica.Blocks.Sources.RealExpression realExpression22(y=tan.heaPorVol[16].T)
    annotation (Placement(transformation(extent={{1276,-688},{1296,-668}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=true)
    annotation (Placement(transformation(extent={{978,-18},{998,2}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y=true)
    annotation (Placement(transformation(extent={{1254,-18},{1274,2}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression2(y=true)
    annotation (Placement(transformation(extent={{1282,-750},{1302,-730}})));
  Modelica.Blocks.Sources.IntegerExpression integerExpression(y=
        chiller_tes_plant_controller.tesStatus)
    annotation (Placement(transformation(extent={{1292,-814},{1312,-794}})));
  Modelica.Blocks.Sources.RealExpression realExpression10(y=Sensor_TCWS.T)
    annotation (Placement(transformation(extent={{1112,-862},{1132,-842}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput cooling_tower_supply_water_temperature
    annotation (Placement(transformation(extent={{1230,-878},{1270,-838}})));
  Modelica.Blocks.Sources.RealExpression realExpression23(y=Sensor_TCWR.T)
    annotation (Placement(transformation(extent={{1118,-926},{1138,-906}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput cooling_tower_return_water_temperature
    annotation (Placement(transformation(extent={{1236,-942},{1276,-902}})));
  Modelica.Blocks.Sources.RealExpression realExpression24(y=cooTow.PFan)
    annotation (Placement(transformation(extent={{1120,-1010},{1140,-990}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput cooling_tower_electrical_power
    annotation (Placement(transformation(extent={{1238,-1026},{1278,-986}})));
  Modelica.Blocks.Sources.RealExpression realExpression25(y=cooTow.Q_flow)
    annotation (Placement(transformation(extent={{1128,-1076},{1148,-1056}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput cooling_tower_thermal_power
    annotation (Placement(transformation(extent={{1246,-1092},{1286,-1052}})));
  Subsequences.cooling_load_ScaleDown_controller
    cooling_load_ScaleDown_controller
    annotation (Placement(transformation(extent={{446,-4},{466,16}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{578,-42},{598,-22}})));
  Subsequences.pump_flow_controller pump_flow_controller(
    mEva1_flow_nominal=per1.mEva_flow_nominal,
    mEva2_flow_nominal=per2.mEva_flow_nominal,
    mCon1_flow_nominal=per1.mCon_flow_nominal,
    mCon2_flow_nominal=per2.mCon_flow_nominal,
    mBypass_flow_nominal=10*0.9997,
    mTankDischarge_flow_nominal=58*0.9997)
    annotation (Placement(transformation(extent={{-258,254},{-238,274}})));
  Subsequences.hx_water_pid_controller hx_water_pid_controller
    annotation (Placement(transformation(extent={{658,-38},{678,-18}})));
  Subsequences.load_request_controller load_request_controller
    annotation (Placement(transformation(extent={{716,-40},{736,-20}})));
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
  connect(conPID1.y, cooTow.y) annotation (Line(points={{-393,66},{-380,66},{-380,
          38},{-353.8,38}}, color={0,0,127}));
  connect(tesStatusController.TesMode, chiller_tes_plant_controller.tesStatus)
    annotation (Line(points={{-382,268},{-382,267.6},{-356.2,267.6}},
                          color={255,127,0}));
  connect(fixHeaFlo.port,vol.heatPort)
    annotation (Line(points={{504,6},{536,6},{536,-32}},      color={191,0,0}));
  connect(cooTow.port_b, Sensor_mCW.port_a) annotation (Line(points={{-290,14},
          {-288,18},{-278,18}}, color={0,127,255}));
  connect(junCwPumRet.port_2, pumCW1.port_a) annotation (Line(points={{-198,36},
          {-184,36},{-184,44},{-170,44}},
                                        color={0,127,255}));
  connect(junCwPumRet.port_3, pumCW2.port_a) annotation (Line(points={{-208,26},
          {-208,12},{-176,12},{-176,-6},{-166,-6}},
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
  connect(mulSum.y, gai6.u) annotation (Line(points={{424,-310},{456,-310}},
                             color={0,0,127}));
  connect(tempTan.T,mulSum. u)
    annotation (Line(points={{369,-310},{400,-310}},
                                                color={0,0,127}));
  connect(tan.heaPorVol,tempTan. port) annotation (Line(points={{263,-21},{328,
          -21},{328,-310},{348,-310}},       color={191,0,0}));
  connect(tempTan[3].T, tesStatusController.TesTopTemp) annotation (Line(points={{369,
          -310},{384,-310},{384,424},{-432,424},{-432,271.8},{-406.2,271.8}},
                         color={0,0,127}));
  connect(tempTan[14].T, tesStatusController.TesBottomTemp) annotation (Line(
        points={{369,-310},{384,-310},{384,424},{-424,424},{-424,261.8},{-406,
          261.8}},                                                        color
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
  connect(systemCommand, chiller_tes_plant_controller.systemCommand)
    annotation (Line(points={{-526,244},{-526,248},{-356,248},{-356,277.8}},
        color={255,127,0}));
  connect(junSecSup.port_2, Sensor_TCHWS.port_a) annotation (Line(points={{272,90},
          {302,90}},                       color={0,127,255}));
  connect(Sensor_TCHWS.port_b, Pump_CHW_Secondary.port_a)
    annotation (Line(points={{322,90},{340,90},{340,88}}, color={0,127,255}));
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
  connect(Sensor_TCWR.port_b, cooTow.port_a) annotation (Line(points={{-336,
          -128},{-366,-128},{-366,14},{-348,14}}, color={0,127,255}));
  connect(Sensor_TCWR.port_a, expVesChi.port_a) annotation (Line(points={{-316,
          -128},{-246,-128},{-246,-105}}, color={0,127,255}));
  connect(Sensor_TCWS.port_a, Sensor_mCW.port_b) annotation (Line(points={{-258,
          56},{-268,56},{-268,36},{-252,36},{-252,18},{-258,18}}, color={0,127,
          255}));
  connect(Sensor_TCWS.port_b, junCwPumRet.port_1) annotation (Line(points={{
          -238,56},{-228,56},{-228,36},{-218,36}}, color={0,127,255}));
  connect(SP_TCWS.y, conPID1.u_s) annotation (Line(points={{-453,46},{-428,46},
          {-428,66},{-416,66}}, color={0,0,127}));
  connect(conPID1.u_m, Sensor_TCWS.T) annotation (Line(points={{-404,54},{-404,
          48},{-368,48},{-368,80},{-248,80},{-248,67}}, color={0,0,127}));
  connect(realExpression12.y, building_cooling_load_actually_served)
    annotation (Line(points={{993,52},{1024,52},{1024,62},{1060,62}}, color={0,
          0,127}));
  connect(booleanExpression.y, chiller_1_OnOff) annotation (Line(points={{999,
          -8},{1000,-12},{1066,-12}}, color={255,0,255}));
  connect(booleanExpression1.y, chiller_2_OnOff) annotation (Line(points={{1275,
          -8},{1280,-4},{1334,-4}}, color={255,0,255}));
  connect(realExpression11.y, chiller_1_supply_water_temperature) annotation (
      Line(points={{973,-106},{1040,-106},{1040,-96},{1074,-96},{1074,-92}},
        color={0,0,127}));
  connect(realExpression9.y, chiller_1_flow_rate) annotation (Line(points={{981,
          -158},{1032,-158},{1032,-168},{1066,-168},{1066,-164}}, color={0,0,
          127}));
  connect(realExpression8.y, chiller_1_thermal_power) annotation (Line(points={
          {971,-244},{971,-248},{1040,-248},{1040,-238},{1076,-238}}, color={0,
          0,127}));
  connect(realExpression7.y, chiller_1_electrical_power) annotation (Line(
        points={{981,-308},{981,-312},{1076,-312}}, color={0,0,127}));
  connect(realExpression6.y, chillers_supply_water_temperature) annotation (
      Line(points={{977,-384},{1040,-384},{1040,-378},{1078,-378}}, color={0,0,
          127}));
  connect(realExpression5.y, chillers_return_water_temperature) annotation (
      Line(points={{977,-454},{1040,-454},{1040,-446},{1076,-446}}, color={0,0,
          127}));
  connect(realExpression4.y, bypass_flow_rate) annotation (Line(points={{971,
          -508},{971,-512},{1040,-512},{1040,-492},{1084,-492}}, color={0,0,127}));
  connect(realExpression3.y, tank_flow_rate) annotation (Line(points={{979,-576},
          {1056,-576},{1056,-568},{1088,-568},{1088,-564}}, color={0,0,127}));
  connect(realExpression2.y, tank_supply_water_temperature)
    annotation (Line(points={{969,-640},{1078,-640}}, color={0,0,127}));
  connect(realExpression1.y, tank_return_water_temperature) annotation (Line(
        points={{965,-698},{1048,-698},{1048,-704},{1082,-704}}, color={0,0,127}));
  connect(realExpression13.y, chiller_2_supply_water_temperature) annotation (
      Line(points={{1275,-76},{1275,-80},{1312,-80},{1312,-68},{1346,-68}},
        color={0,0,127}));
  connect(realExpression14.y, chiller_2_flow_rate) annotation (Line(points={{
          1281,-132},{1281,-136},{1312,-136},{1312,-148},{1354,-148}}, color={0,
          0,127}));
  connect(realExpression15.y, chiller_2_thermal_power) annotation (Line(points=
          {{1283,-224},{1288,-222},{1364,-222}}, color={0,0,127}));
  connect(realExpression16.y, chiller_2_electrical_power) annotation (Line(
        points={{1281,-288},{1328,-288},{1328,-296},{1364,-296}}, color={0,0,
          127}));
  connect(realExpression17.y, tank_average_temperature) annotation (Line(points
        ={{1287,-382},{1296,-380},{1380,-380}}, color={0,0,127}));
  connect(realExpression18.y, tank_top_temperature) annotation (Line(points={{
          1285,-444},{1285,-448},{1344,-448},{1344,-436},{1378,-436}}, color={0,
          0,127}));
  connect(realExpression19.y, tank_layer_3_temperature) annotation (Line(points
        ={{1287,-512},{1296,-508},{1378,-508}}, color={0,0,127}));
  connect(realExpression20.y, tank_layer_9_temperature) annotation (Line(points
        ={{1293,-582},{1352,-582},{1352,-578},{1386,-578}}, color={0,0,127}));
  connect(realExpression21.y, tank_layer_14_temperature) annotation (Line(
        points={{1289,-622},{1296,-624},{1378,-624}}, color={0,0,127}));
  connect(realExpression22.y, tank_bottom_temperature) annotation (Line(points=
          {{1297,-678},{1352,-678},{1352,-658},{1386,-658}}, color={0,0,127}));
  connect(booleanExpression2.y, building_cooling_load_request) annotation (Line(
        points={{1303,-740},{1303,-744},{1352,-744},{1352,-738},{1384,-738}},
        color={255,0,255}));
  connect(integerExpression.y, tank_status) annotation (Line(points={{1313,-804},
          {1342.5,-804},{1342.5,-800},{1388,-800}}, color={255,127,0}));
  connect(realExpression10.y, cooling_tower_supply_water_temperature)
    annotation (Line(points={{1133,-852},{1216,-852},{1216,-858},{1250,-858}},
        color={0,0,127}));
  connect(realExpression23.y, cooling_tower_return_water_temperature)
    annotation (Line(points={{1139,-916},{1222,-916},{1222,-922},{1256,-922}},
        color={0,0,127}));
  connect(realExpression24.y, cooling_tower_electrical_power) annotation (Line(
        points={{1141,-1000},{1224,-1000},{1224,-1006},{1258,-1006}}, color={0,
          0,127}));
  connect(realExpression25.y, cooling_tower_thermal_power) annotation (Line(
        points={{1149,-1066},{1232,-1066},{1232,-1072},{1266,-1072}}, color={0,
          0,127}));
  connect(temperatureSensor.T, cooling_load_ScaleDown_controller.hx_water_temperature)
    annotation (Line(points={{599,-32},{616,-32},{616,38},{410,38},{410,11.6},{
          444,11.6}},                                                  color={0,
          0,127}));
  connect(vol.heatPort, temperatureSensor.port) annotation (Line(points={{536,-32},
          {578,-32}},                          color={191,0,0}));
  connect(chiller_tes_plant_controller.systemMode, pump_flow_controller.systemMode)
    annotation (Line(points={{-332,270},{-328,269.657},{-259.176,269.657}},
                                                                 color={255,127,
          0}));
  connect(chi1.on, pump_flow_controller.chiller1On) annotation (Line(points={{-53,0},
          {-53,8},{-40,8},{-40,40},{-16,40},{-16,288},{-236.941,288},{-236.941,
          272.857}},                               color={255,0,255}));
  connect(chi2.on, pump_flow_controller.chiller2On) annotation (Line(points={{-89,0},
          {-89,16},{-112,16},{-112,120},{-312,120},{-312,240},{-236.941,240},{
          -236.941,270.114}},
        color={255,0,255}));
  connect(pumCW2.m_flow_in, pump_flow_controller.mCon2_flow) annotation (Line(
        points={{-156,-18},{-160,-18},{-160,-32},{-128,-32},{-128,96},{-96,96},
          {-96,280},{-224,280},{-224,272},{-236.941,272},{-236.941,262.343}},
                                                                     color={0,0,
          127}));
  connect(Pump_CHW_Primary2.m_flow_in, pump_flow_controller.mEva2_flow)
    annotation (Line(points={{-92,-82},{-120,-82},{-120,88},{-328,88},{-328,208},
          {-216,208},{-216,256},{-236.941,256},{-236.941,266.229}},
        color={0,0,127}));
  connect(pumCW1.m_flow_in, pump_flow_controller.mCon1_flow) annotation (Line(
        points={{-160,32},{-184,32},{-184,104},{-128,104},{-128,128},{-176,128},
          {-176,264.171},{-236.941,264.171}},
                   color={0,0,127}));
  connect(Pump_CHW_Primary1.m_flow_in, pump_flow_controller.mEva1_flow)
    annotation (Line(points={{-56,-82},{-56,-56},{-8,-56},{-8,268.057},{
          -236.941,268.057}},                                        color={0,0,
          127}));
  connect(Pump_CHW_Bypass.m_flow_in, pump_flow_controller.mBypass_flow)
    annotation (Line(points={{122,-54},{136,-54},{136,312},{-248,312},{-248,280},
          {-236.941,280},{-236.941,260.171}},
                   color={0,0,127}));
  connect(Pump_CHW_Secondary.m_flow_in, pump_flow_controller.mSecondary_flow)
    annotation (Line(points={{350,100},{350,320},{-256,320},{-256,280},{
          -236.824,280},{-236.824,257.429}},            color={0,0,127}));
  connect(cooling_load_ScaleDown_controller.building_cooling_load_actual,
    fixHeaFlo.Q_flow)
    annotation (Line(points={{468,6},{484,6}}, color={0,0,127}));
  connect(building_cooling_load, cooling_load_ScaleDown_controller.building_cooling_load)
    annotation (Line(points={{-514,108},{-514,112},{-384,112},{-384,88},{-376,
          88},{-376,-184},{400,-184},{400,0.6},{444,0.6}}, color={0,0,127}));
  connect(con6.y, hx_water_pid_controller.hx_water_temperature_setpoint)
    annotation (Line(points={{581.8,9},{640,9},{640,-22.6},{656,-22.6}}, color=
          {0,0,127}));
  connect(temperatureSensor.T, hx_water_pid_controller.hx_water_temperature)
    annotation (Line(points={{599,-32},{599,-32.8},{656,-32.8}}, color={0,0,127}));
  connect(hx_water_pid_controller.hx_water_pid_out, pump_flow_controller.hx_water_pid)
    annotation (Line(points={{680,-28},{696,-28},{696,160},{360,160},{360,328},
          {-272,328},{-272,259.257},{-259.059,259.257}}, color={0,0,127}));
  connect(hx_water_pid_controller.hx_water_pid_out, load_request_controller.hx_water_pid)
    annotation (Line(points={{680,-28},{680,-35.6},{714.4,-35.6}}, color={0,0,
          127}));
  connect(outside_air_temperature, load_request_controller.outside_air_temperature)
    annotation (Line(points={{-534,186},{-536,186},{-536,440},{714,440},{714,
          -24}}, color={0,0,127}));
  connect(load_request_controller.loadRequest, chiller_tes_plant_controller.loadRequest)
    annotation (Line(points={{738,-30},{764,-30},{764,176},{368,176},{368,336},
          {-356,336},{-356,273.8}}, color={255,0,255}));
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
end campus_chiller_plant;
