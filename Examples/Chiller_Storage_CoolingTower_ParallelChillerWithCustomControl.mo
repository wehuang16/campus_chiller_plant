within campus_chiller_plant.Examples;
model Chiller_Storage_CoolingTower_ParallelChillerWithCustomControl
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
        origin={22,-12})));
  Buildings.Fluid.Movers.FlowControlled_m_flow Pump_CHW_Primary1(
    redeclare package Medium = ChilledWater,
    addPowerToMedium=false,
    m_flow_nominal=mEva_flow_nominal,
    tau=30,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    "Pump for chilled water loop" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={44,-118})));
  Modelica.Blocks.Sources.Constant SP_TCHe[2](k={273.15 + 5.28,273.15 + 4.17})
    "chilled water suppy temperature setpoint"
                              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-76,62})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_mS(redeclare package Medium =
        ChilledWater)                                                                   annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={184,-22})));
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
        origin={246,10})));
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
    addPowerToMedium=false,
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
        rotation=0,
        origin={299,-35})));
  BaseClasses.chiller_tes_plant_controller_ParallelChillerWithCustomControl chiller_tes_plant_controller
    annotation (Placement(transformation(extent={{170,184},{190,204}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.TimeTable all_weekday_daily_schedule(
    table=[0,2; 6,1; 14,3; 22,2; 24,2],
    timeScale=3600,
    period(displayUnit="s") = 86400)
    annotation (Placement(transformation(extent={{34,182},{54,202}})));
  BaseClasses.TesStatusController
                      tesStatusController(tempDischargedBottom(displayUnit=
          "degC") = 285.93, tempChargedTop(displayUnit="degC") = 278.71)
    annotation (Placement(transformation(extent={{260,190},{280,210}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis hys(uLow=0.05, uHigh=0.1)
    annotation (Placement(transformation(extent={{442,116},{462,136}})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai(k=mEva_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={166,-138})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai1(k=mEva_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={332,70})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow fixHeaFlo(T_ref=
        293.15)
    "Fixed heat flow rate"
    annotation (Placement(transformation(extent={{590,36},{610,56}})));
  Buildings.Fluid.MixingVolumes.MixingVolume vol(
    T_start=283.71,
    nPorts=3,
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
    annotation (Placement(transformation(extent={{718,-140},{738,-120}})));
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
    addPowerToMedium=false,
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
        origin={22,90})));
  Buildings.Fluid.Sensors.MassFlowRate Sensor_mCHi(redeclare package Medium =
        ChilledWater)                                                                     annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={114,-118})));
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
        origin={198,-58})));
  Buildings.Controls.OBC.CDL.Reals.Switch swi
    annotation (Placement(transformation(extent={{228,-172},{248,-152}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con(k=0)
    annotation (Placement(transformation(extent={{194,-192},{214,-172}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea1(realTrue=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={62,-38})));
  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter gai4(k=57.08*0.9997)
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
  Buildings.Controls.OBC.CDL.Reals.Line lin
    annotation (Placement(transformation(extent={{560,134},{580,154}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con2(k=273.15 + 11.11)
    annotation (Placement(transformation(extent={{512,230},{532,250}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con3(k=1)
    annotation (Placement(transformation(extent={{506,164},{526,184}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con4(k=273.15 + 11.67)
    annotation (Placement(transformation(extent={{524,116},{544,136}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con5(k=0)
    annotation (Placement(transformation(extent={{530,74},{550,94}})));
  Buildings.Controls.OBC.CDL.Reals.Multiply loaAct annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={760,-56})));
  Buildings.Controls.Continuous.LimPID conPID(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.3,
    Ti=150,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    y_start=0.2,
    reverseActing=false)
    annotation (Placement(transformation(extent={{494,-18},{514,2}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con6(k=273.15 + 10.56)
    annotation (Placement(transformation(extent={{446,-12},{464,6}})));
  Modelica.Blocks.Sources.CombiTimeTable dataChiller(
    tableOnFile=true,
    tableName="tab1",
    fileName=ModelicaServices.ExternalReferences.loadResource(
        "modelica://campus_chiller_plant/Resources/chiller_trend_updated.txt"),
    columns=2:11,
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint)
    annotation (Placement(transformation(extent={{664,-80},{684,-60}})));

  Buildings.Controls.OBC.CDL.Reals.MultiplyByParameter OAT(k=1) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={670,180})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis OAT_lockout(uLow=19.95, uHigh=20)
    annotation (Placement(transformation(extent={{620,214},{640,234}})));
  Buildings.Controls.OBC.CDL.Logical.And and2
    annotation (Placement(transformation(extent={{444,270},{464,290}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor tempTan[16]
    " tank tempearture" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={446,-184})));
  Buildings.Controls.OBC.CDL.Reals.MultiSum mulSum(nin=16)
    annotation (Placement(transformation(extent={{502,-194},{522,-174}})));
  Modelica.Blocks.Math.Gain gai6(k=1/16) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={564,-188})));
  Buildings.Controls.OBC.CDL.Integers.Sources.TimeTable weekly_schedule(
    table=[0,2; 5.75,0; 6,1; 13.75,0; 14,3; 21.5,0; 21.75,2; 24,2; 29.75,0; 30,
        1; 37.75,0; 38,3; 45.5,0; 45.75,2; 48,2; 53.75,0; 54,1; 61.75,0; 62,3;
        69.5,0; 69.75,2; 72,2; 78.5,0; 78.8333,1; 89.5,0; 96,2; 109.8333,0; 116,
        2; 120,2; 124,0; 124.25,1; 133.75,0; 134,3; 141.5,0; 141.75,2; 144,2;
        149.75,0; 150,1; 157.75,0; 158,3; 165.5,0; 165.75,2; 168,2],
    timeScale=3600,
    period=86400*7)
    annotation (Placement(transformation(extent={{108,174},{128,194}})));
  Buildings.Controls.OBC.UnitConversions.To_degC tank_average_temperature_simulation
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={608,-182})));
  Buildings.Controls.OBC.CDL.Integers.Sources.TimeTable custom_schedule(
    table=[0,2; 1992,2; 1998,1; 2006,3; 2013.8333,0; 2014.1667,2; 2016,2; 2022,
        1; 2030,3; 2037.8333,0; 2038.1667,2; 2040,2; 2046,1; 2050.6667,3;
        2061.8333,0; 2062.1667,2; 2064,2; 2070,1; 2078,3; 2085.8333,0;
        2086.1667,2; 2088,2; 2094,1; 2102,3; 2109.8333,0; 2110.1667,0; 2112,0;
        2118,1; 2126,3; 2133.8333,0; 2134.1667,0; 2136,0; 2142,1; 2145.75,3;
        2157.8333,0; 2158.1667,2; 2160,2; 2166,1; 2174,3; 2181.8333,0;
        2182.1667,2; 2184,2; 2190,1; 2198,3; 2205.8333,0; 2206.1667,2; 2208,2;
        2214,1; 2222,3; 2229.8333,0; 2230.1667,2; 2232,2; 2238,1; 2246,3;
        2253.8333,0; 2254.1667,2; 2256,2; 2262,1; 2270,3; 2277.8333,0;
        2278.1667,2; 2280,2; 2286,1; 2294,3; 2301.8333,0; 2302.1667,2; 2304,2;
        2310,1; 2318,3; 2325.8333,0; 2326.1667,2; 2328,2; 2334,1; 2342,3;
        2349.8333,0; 2350.1667,2; 2352,2; 2358,1; 2366,3; 2373.8333,0;
        2374.1667,2; 2376,2; 2382,1; 2390,3; 2397.8333,0; 2398.1667,2; 2400,2;
        2406,1; 2414,3; 2421.8333,0; 2422.1667,2; 2424,2; 2430,1; 2438,3;
        2445.8333,0; 2446.1667,2; 2448,2; 2454,1; 2462,3; 2469.8333,0;
        2470.1667,2; 2472,2; 2478,1; 2486,3; 2493.8333,0; 2494.1667,2; 2496,2],
    timeScale=3600,
    period(displayUnit="d") = 31536000)
    annotation (Placement(transformation(extent={{74,150},{94,170}})));

  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_TCHW_supply_1(redeclare
      package Medium = ChilledWater, m_flow_nominal=mEva_flow_nominal,
    T_start=278.43)
    annotation (Placement(transformation(extent={{-46,38},{-26,58}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_TCHW_supply_2(redeclare
      package Medium = ChilledWater, m_flow_nominal=mEva_flow_nominal,
    T_start=277.32)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={26,56})));
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
        origin={80,-160})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_Ttankout(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    T_start=277.32)
    annotation (Placement(transformation(extent={{242,-62},{262,-42}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_Ttankin(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    T_start=283.71)
    annotation (Placement(transformation(extent={{308,-18},{328,2}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort Sensor_TCHWR(redeclare package
      Medium = ChilledWater, m_flow_nominal=mEva_flow_nominal) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={410,-84})));
  Buildings.Controls.OBC.CDL.Conversions.RealToInteger reaToInt
    annotation (Placement(transformation(extent={{96,220},{116,240}})));
  Buildings.HeatTransfer.Sources.PrescribedHeatFlow heat_gain_all[16]
    annotation (Placement(transformation(extent={{256,-40},{276,-20}})));
  Buildings.HeatTransfer.Sources.PrescribedHeatFlow heat_gain_top
    annotation (Placement(transformation(extent={{264,-24},{284,-4}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con7(k=305.01)
    annotation (Placement(transformation(extent={{230,-24},{250,-4}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant con8[16](k=0)
    annotation (Placement(transformation(extent={{208,-104},{228,-84}})));
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
  connect(junSecSup.port_3, Sensor_pSupper.port) annotation (Line(points={{178,18},
          {184,18},{184,10},{236,10}},   color={0,127,255}));
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
  connect(Sensor_psub_Suction.port, Pump_CHW_Secondary.port_a) annotation (Line(
        points={{230,32},{230,24},{328,24},{328,32},{340,32}}, color={0,127,255}));
  connect(tesStatusController.TesMode, chiller_tes_plant_controller.tesStatus)
    annotation (Line(points={{282,200},{288,200},{288,220},{156,220},{156,191.6},
          {167.8,191.6}}, color={255,127,0}));
  connect(fixHeaFlo.port,vol.heatPort)
    annotation (Line(points={{610,46},{620,46},{620,4},{560,4},{560,-18},{568,
          -18}},                                              color={191,0,0}));
  connect(res2.port_b, vol.ports[1]) annotation (Line(points={{438,30},{524,30},
          {524,-40},{576.667,-40},{576.667,-28}},   color={0,127,255}));
  connect(bou.ports[1], vol.ports[2]) annotation (Line(points={{470,-116},{578,
          -116},{578,-28}},                           color={0,127,255}));
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
  connect(gai.u, gai3.u) annotation (Line(points={{178,-138},{178,-188},{-126,
          -188},{-126,-84}},       color={0,0,127}));
  connect(junChwPumRet.port_2, Pump_CHW_Primary1.port_a) annotation (Line(
        points={{70,-104},{64,-104},{64,-118},{54,-118}}, color={0,127,255}));
  connect(junChwPumRet.port_3, Pump_CHW_Primary2.port_a) annotation (Line(
        points={{80,-94},{65,-94},{65,-72},{54,-72}}, color={0,127,255}));
  connect(Pump_CHW_Primary2.port_b, chi2.port_a2) annotation (Line(points={{34,
          -72},{28,-72},{28,-70},{-4,-70},{-4,-22}}, color={0,127,255}));
  connect(Pump_CHW_Primary1.port_b, chi1.port_a2) annotation (Line(points={{34,
          -118},{28,-118},{28,-22}}, color={0,127,255}));
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
  connect(chiller_tes_plant_controller.chiller1On, swi.u2) annotation (Line(
        points={{192,200},{216,200},{216,180},{204,180},{204,92},{156,92},{156,
          -4},{204,-4},{204,-36},{224,-36},{224,-80},{320,-80},{320,-204},{188,
          -204},{188,-162},{226,-162}},
        color={255,0,255}));
  connect(con.y, swi.u3) annotation (Line(points={{216,-182},{216,-180},{226,
          -180},{226,-170}},
                       color={0,0,127}));
  connect(gai.y, Pump_CHW_Primary1.m_flow_in) annotation (Line(points={{154,
          -138},{44,-138},{44,-130}},
                                color={0,0,127}));
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
  connect(realExpression.y, lin.u) annotation (Line(points={{479,-58},{484,-58},
          {484,146},{558,146},{558,144}}, color={0,0,127}));
  connect(con2.y, lin.x1) annotation (Line(points={{534,240},{548,240},{548,152},
          {558,152}}, color={0,0,127}));
  connect(con3.y, lin.f1) annotation (Line(points={{528,174},{536,174},{536,144},
          {558,144},{558,148}}, color={0,0,127}));
  connect(con4.y, lin.x2) annotation (Line(points={{546,126},{548,126},{548,140},
          {558,140}}, color={0,0,127}));
  connect(con5.y, lin.f2)
    annotation (Line(points={{552,84},{558,84},{558,136}}, color={0,0,127}));
  connect(lin.y, loaAct.u1) annotation (Line(points={{582,144},{650,144},{650,
          136},{748,136},{748,-50}}, color={0,0,127}));
  connect(loaAct.y, fixHeaFlo.Q_flow) annotation (Line(points={{772,-56},{780,
          -56},{780,-108},{628,-108},{628,68},{584,68},{584,46},{590,46}},
        color={0,0,127}));
  connect(realExpression.y, conPID.u_m) annotation (Line(points={{479,-58},{479,
          -60},{484,-60},{484,-28},{504,-28},{504,-20}}, color={0,0,127}));
  connect(con6.y, conPID.u_s) annotation (Line(points={{465.8,-3},{465.8,0},{
          476,0},{476,-8},{492,-8}},
                              color={0,0,127}));
  connect(conPID.y, hys.u) annotation (Line(points={{515,-8},{515,92},{440,92},
          {440,126}}, color={0,0,127}));
  connect(swi1.y, gai1.u) annotation (Line(points={{290,78},{308,78},{308,70},{
          320,70}}, color={0,0,127}));
  connect(conPID.y, swi1.u1) annotation (Line(points={{515,-8},{528,-8},{528,
          104},{264,104},{264,86},{266,86}}, color={0,0,127}));
  connect(swi.y, gai.u) annotation (Line(points={{250,-162},{272,-162},{272,
          -138},{178,-138}}, color={0,0,127}));
  connect(conPID.y, swi.u1) annotation (Line(points={{515,-8},{542,-8},{542,
          -132},{226,-132},{226,-154}}, color={0,0,127}));
  connect(dataChiller.y[6], loaAct.u2) annotation (Line(points={{685,-70},{685,
          -72},{740,-72},{740,-62},{748,-62}}, color={0,0,127}));
  connect(dataChiller.y[9], OAT.u)
    annotation (Line(points={{685,-70},{685,180},{682,180}}, color={0,0,127}));
  connect(OAT_lockout.u, OAT.y) annotation (Line(points={{618,224},{604,224},{
          604,202},{658,202},{658,180}}, color={0,0,127}));
  connect(OAT_lockout.y, and2.u1) annotation (Line(points={{642,224},{658,224},
          {658,234},{668,234},{668,324},{442,324},{442,280}}, color={255,0,255}));
  connect(hys.y, and2.u2) annotation (Line(points={{464,126},{472,126},{472,264},
          {442,264},{442,272}}, color={255,0,255}));
  connect(mulSum.y, gai6.u) annotation (Line(points={{524,-184},{542,-184},{542,
          -188},{552,-188}}, color={0,0,127}));
  connect(tempTan.T,mulSum. u)
    annotation (Line(points={{457,-184},{500,-184}},
                                                color={0,0,127}));
  connect(tan.heaPorVol,tempTan. port) annotation (Line(points={{299,-35},{382,
          -35},{382,-168},{430,-168},{430,-184},{436,-184}},
                                             color={191,0,0}));
  connect(tempTan[3].T, tesStatusController.TesTopTemp) annotation (Line(points={{457,
          -184},{480,-184},{480,-76},{488,-76},{488,-32},{472,-32},{472,108},{
          260,108},{260,192},{248,192},{248,203.8},{257.8,203.8}},
                         color={0,0,127}));
  connect(tempTan[14].T, tesStatusController.TesBottomTemp) annotation (Line(
        points={{457,-184},{480,-184},{480,-76},{488,-76},{488,-32},{472,-32},{
          472,108},{258,108},{258,193.8}},                                color
        ={0,0,127}));
  connect(gai6.y, tank_average_temperature_simulation.u) annotation (Line(
        points={{575,-188},{588,-188},{588,-182},{596,-182}}, color={0,0,127}));
  connect(Sensor_TCHW_supply_1.port_a, chi1.port_b2) annotation (Line(points={{
          -46,48},{-56,48},{-56,28},{-12,28},{-12,40},{8,40},{8,32},{28,32},{28,
          -2}}, color={0,127,255}));
  connect(Sensor_TCHW_supply_1.port_b, junChwPumSup.port_1) annotation (Line(
        points={{-26,48},{0,48},{0,90},{12,90}}, color={0,127,255}));
  connect(junChwPumSup.port_3, Sensor_TCHW_supply_2.port_b)
    annotation (Line(points={{22,80},{26,80},{26,66}}, color={0,127,255}));
  connect(Sensor_TCHW_supply_2.port_a, chi2.port_b2) annotation (Line(points={{
          26,46},{28,46},{28,36},{32,36},{32,28},{-4,28},{-4,-2}}, color={0,127,
          255}));
  connect(Sensor_mCHi.port_b, Sensor_TCHW_primary_return.port_a) annotation (
      Line(points={{104,-118},{104,-160},{90,-160}}, color={0,127,255}));
  connect(junChwPumRet.port_1, Sensor_TCHW_primary_return.port_b) annotation (
      Line(points={{90,-104},{90,-144},{64,-144},{64,-160},{70,-160}}, color={0,
          127,255}));
  connect(Sensor_mS.port_b, Sensor_Ttankout.port_a) annotation (Line(points={{
          184,-32},{184,-40},{236,-40},{236,-52},{242,-52}}, color={0,127,255}));
  connect(Sensor_Ttankout.port_b, tan.port_b) annotation (Line(points={{262,-52},
          {299,-52},{299,-48}}, color={0,127,255}));
  connect(junChwPumSup.port_2, Sensor_TCHW_primary_supply.port_a)
    annotation (Line(points={{32,90},{54,90}}, color={0,127,255}));
  connect(Sensor_TCHW_primary_supply.port_b, junPriSup.port_1) annotation (Line(
        points={{74,90},{84,90},{84,70},{90,70}}, color={0,127,255}));
  connect(vol.ports[3], Sensor_TCHWR.port_a) annotation (Line(points={{579.333,
          -28},{579.333,-40},{500,-40},{500,-84},{420,-84}}, color={0,127,255}));
  connect(Sensor_TCHWR.port_b, junSecRet.port_1) annotation (Line(points={{400,
          -84},{360,-84},{360,-82},{348,-82}}, color={0,127,255}));
  connect(res3.port_a, Sensor_Ttankin.port_b)
    annotation (Line(points={{340,-16},{340,-8},{328,-8}}, color={0,127,255}));
  connect(Sensor_Ttankin.port_a, tan.port_a)
    annotation (Line(points={{308,-8},{299,-8},{299,-22}}, color={0,127,255}));
  connect(and2.u2, chiller_tes_plant_controller.loadRequest) annotation (Line(
        points={{442,272},{324,272},{324,274},{168,274},{168,197.8}}, color={
          255,0,255}));
  connect(reaToInt.y, chiller_tes_plant_controller.systemCommand) annotation (
      Line(points={{118,230},{132,230},{132,204},{160,204},{160,201.8},{168,
          201.8}}, color={255,127,0}));
  connect(dataChiller.y[10], reaToInt.u) annotation (Line(points={{685,-70},{
          848,-70},{848,308},{94,308},{94,230}}, color={0,0,127}));
  connect(heat_gain_all.port, tan.heaPorVol) annotation (Line(points={{276,-30},
          {276,-28},{284,-28},{284,-35},{299,-35}}, color={191,0,0}));
  connect(heat_gain_top.port, tan.heaPorVol[1]) annotation (Line(points={{284,-14},
          {284,-16},{292,-16},{292,20},{228,20},{228,-72},{299,-72},{299,
          -35.3656}}, color={191,0,0}));
  connect(con7.y, heat_gain_top.Q_flow)
    annotation (Line(points={{252,-14},{264,-14}}, color={0,0,127}));
  connect(con8.y, heat_gain_all.Q_flow) annotation (Line(points={{230,-94},{240,
          -94},{240,-76},{236,-76},{236,-56},{232,-56},{232,-30},{256,-30}},
        color={0,0,127}));
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
end Chiller_Storage_CoolingTower_ParallelChillerWithCustomControl;
