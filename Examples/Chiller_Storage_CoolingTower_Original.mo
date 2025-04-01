within campus_chiller_plant.Examples;
model Chiller_Storage_CoolingTower_Original "Put another chiller"
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
  Modelica.Blocks.Logical.GreaterThreshold greaterThreshold(threshold=0)
    annotation (Placement(transformation(extent={{-86,28},{-66,48}})));
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
    QEva_flow_nominal(displayUnit="kW") = -3000000,
    COP_nominal=7,
    PLRMax=1,
    PLRMinUnl=0.4,
    mEva_flow_nominal=1000*2*0.06,
    mCon_flow_nominal=1000*3*0.06,
    capFunT={0.70790824,-0.002006568,-0.00259605,0.030058776,-0.0010564344,0.0020457036},
    EIRFunT={0.5605438,-0.01377927,6.57072e-005,0.013219362,0.000268596,-0.0005011308},
    EIRFunPLR={0.17149273,0.58820208,0.23737257}) "Chiller performance data"
    annotation (Placement(transformation(extent={{304,92},{346,134}})));

  Modelica.Blocks.Sources.Constant ChillerON(k=1) "chilled water flow rate"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-122,38})));
  Buildings.Fluid.Chillers.ElectricEIR chi1(
    redeclare package Medium1 = CondensorWater,
    redeclare package Medium2 = ChilledWater,
    per=per,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    dp1_nominal=dP0,
    dp2_nominal=dP0) "Chiller model" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={10,-42})));
  Buildings.Fluid.Movers.FlowControlled_m_flow Pump_CHW_Primary(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    tau=30,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    "Pump for chilled water loop" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={62,-94})));
  Modelica.Blocks.Sources.Constant SP_TCHe[2](k={273.15 + 10,273.15 + 4})
    "chilled water flow rate" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-76,62})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCHe(redeclare package Medium =
        ChilledWater)
    annotation (Placement(transformation(extent={{-8,20},{12,40}})));
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
  Buildings.Fluid.Sensors.MassFlowRate Sensor_mCHi(redeclare package Medium =
        ChilledWater)                                                                     annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={16,-78})));
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
        origin={114,-94})));
  Buildings.Fluid.FixedResistances.PressureDrop res3(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    dp_nominal=dP0)                                                                         annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={170,-42})));
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
  Buildings.Fluid.Sensors.Pressure Sensor_pCHe(redeclare package Medium =
        ChilledWater)                                                                  annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={26,30})));
  Buildings.Fluid.Sensors.Pressure Sensor_pCHi(redeclare package Medium =
        ChilledWater)                                         annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={30,-104})));
  Buildings.Fluid.Sensors.Pressure Sensor_psub_Suction(redeclare package Medium
      = ChilledWater)
    annotation (Placement(transformation(extent={{130,20},{150,40}})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCHWS(redeclare package Medium =
        ChilledWater)
    annotation (Placement(transformation(extent={{110,22},{130,42}})));
  Buildings.Fluid.Sources.Boundary_pT ReturnFromBuilding(
    redeclare package Medium = ChilledWater,
    use_T_in=true,
    nPorts=2) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={254,-96})));
  campus_chiller_plant.Examples.BaseClasses.BuildingReturnCal buildingReturnCal annotation (
      Placement(transformation(
        extent={{-15,11},{15,-11}},
        rotation=180,
        origin={291,-95})));
  Modelica.Blocks.Sources.Pulse SP_mCH(
    amplitude=0.9*mEva_flow_nominal,
    width=20,
    period(displayUnit="d") = 86400,
    offset=0.1*mEva_flow_nominal,
    startTime(displayUnit="h") = -18000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={102,-136})));
  Modelica.Blocks.Math.Gain SP_msup_dummys(k=1/(4200*5)) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={236,82})));
  Modelica.Blocks.Sources.Pulse BuildingCoolingLoads(
    amplitude=500*3*1e3,
    width=50,
    period(displayUnit="d") = 86400,
    offset=0.1*500*3*1e3,
    startTime(displayUnit="h") = 25200) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={370,-92})));
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
  Buildings.Fluid.Movers.FlowControlled_m_flow pumCW(
    redeclare package Medium = CondensorWater,
    m_flow_nominal=mCon_flow_nominal,
    dp(start=dP0),
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    "Condenser water pump" annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={-164,14})));
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
        origin={-96,14})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCWS(redeclare package Medium =
        CondensorWater)
    annotation (Placement(transformation(extent={{-56,14},{-36,34}})));
  Buildings.Fluid.Sensors.Temperature Sensor_TCWR(redeclare package Medium =
        CondensorWater)
    annotation (Placement(transformation(extent={{-168,-106},{-148,-86}})));
  Modelica.Blocks.Math.Gain SP_mCWS(k=mCon_flow_nominal/mEva_flow_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-154,-48})));
  Modelica.Blocks.Continuous.LimPID PID(yMax=1, yMin=0)
    annotation (Placement(transformation(extent={{-336,56},{-316,76}})));
  Modelica.Blocks.Sources.Constant SP_TCWS(k=273.15 + 10)
    "chilled water flow rate" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-386,46})));
  Buildings.Fluid.Sources.Boundary_pT bou(redeclare package Medium =
        ChilledWater, nPorts=1)
    annotation (Placement(transformation(extent={{270,8},{290,28}})));
  Buildings.Fluid.Storage.Stratified tan(
    redeclare package Medium = ChilledWater,
    m_flow_nominal=mEva_flow_nominal,
    VTan=1000,
    hTan=15,
    dIns=0.01,
    T_start=278.15,
    tau=600)
    annotation (Placement(transformation(extent={{132,-52},{158,-26}})));
  Buildings.Fluid.Sensors.Temperature Sensor_Ttankin(redeclare package Medium
      = ChilledWater)
    annotation (Placement(transformation(extent={{148,-16},{168,4}})));
  Buildings.Fluid.Sensors.Temperature Sensor_Ttankout(redeclare package Medium
      = ChilledWater)
    annotation (Placement(transformation(extent={{118,-74},{138,-54}})));
equation
  connect(ChillerON.y, greaterThreshold.u)
    annotation (Line(points={{-111,38},{-88,38}},
                                               color={0,0,127}));
  connect(chi1.on, greaterThreshold.y) annotation (Line(
      points={{7,-30},{8,-30},{8,38},{-65,38}},
      color={255,0,255},
      smooth=Smooth.None));
  connect(chi1.port_b1, res1.port_a) annotation (Line(points={{4,-52},{-2,-52},
          {-2,-86},{-8,-86}}, color={0,127,255}));
  connect(Pump_CHW_Secondary.port_b, Sensor_msup.port_a)
    annotation (Line(points={{192,20},{198,20}}, color={0,127,255}));
  connect(Sensor_msup.port_b, res2.port_a)
    annotation (Line(points={{218,20},{230,20}},         color={0,127,255}));
  connect(Pump_CHW_Primary.port_b,Sensor_mCHi. port_a)
    annotation (Line(points={{52,-94},{16,-94},{16,-88}}, color={0,127,255}));
  connect(Sensor_mCHi.port_b, chi1.port_a2)
    annotation (Line(points={{16,-68},{16,-52}}, color={0,127,255}));
  connect(junSu.port_3, Sensor_mS.port_a)
    annotation (Line(points={{88,6},{88,-10},{84,-10},{84,-24},{94,-24}},
                                                        color={0,127,255}));
  connect(junRe.port_2, Pump_CHW_Primary.port_a)
    annotation (Line(points={{104,-94},{72,-94}}, color={0,127,255}));
  connect(res3.port_b, junRe.port_3)
    annotation (Line(points={{170,-52},{170,-84},{114,-84}},
                                                 color={0,127,255}));
  connect(junSu.port_3, Sensor_pSupper.port) annotation (Line(points={{88,6},{
          88,-2},{100,-2}},      color={0,127,255}));
  connect(res3.port_b, Sensor_pSdown.port)
    annotation (Line(points={{170,-52},{170,-62},{186,-62}},
                                                   color={0,127,255}));
  connect(Pump_CHW_Primary.port_b, Sensor_pCHi.port)
    annotation (Line(points={{52,-94},{30,-94}}, color={0,127,255}));
  connect(junSu.port_2, Sensor_psub_Suction.port)
    annotation (Line(points={{98,16},{128,16},{128,14},{140,14},{140,20}},
                                               color={0,127,255}));
  connect(junSu.port_2, Sensor_TCHWS.port)
    annotation (Line(points={{98,16},{120,16},{120,22}},
                                               color={0,127,255}));
  connect(Sensor_msup.m_flow, buildingReturnCal.msup) annotation (Line(points={{208,31},
          {208,54},{314,54},{314,-100.72},{300.3,-100.72}},          color={0,0,
          127}));
  connect(ReturnFromBuilding.ports[1], junRe.port_1) annotation (Line(points={{244,-97},
          {184,-97},{184,-94},{124,-94}},          color={0,127,255}));
  connect(ReturnFromBuilding.ports[2], Sensor_TCHWR.port) annotation (Line(
        points={{244,-95},{238,-95},{238,-94},{230,-94}}, color={0,127,255}));
  connect(SP_mCH.y, Pump_CHW_Primary.m_flow_in)
    annotation (Line(points={{91,-136},{62,-136},{62,-106}}, color={0,0,127}));
  connect(BuildingCoolingLoads.y, buildingReturnCal.QCL) annotation (Line(
        points={{359,-92},{328,-92},{328,-90},{300.3,-90},{300.3,-91.48}},
        color={0,0,127}));
  connect(BuildingCoolingLoads.y, SP_msup_dummys.u) annotation (Line(points={{
          359,-92},{348,-92},{348,82},{248,82}}, color={0,0,127}));
  connect(SP_msup_dummys.y, Pump_CHW_Secondary.m_flow_in)
    annotation (Line(points={{225,82},{218,82},{218,56},{182,56},{182,32}},
                                                          color={0,0,127}));
  connect(Sensor_TCHWS.T, buildingReturnCal.Tsup) annotation (Line(points={{127,32},
          {128,32},{128,60},{322,60},{322,-96.54},{300.3,-96.54}},     color={0,
          0,127}));
  connect(buildingReturnCal.Tret, ReturnFromBuilding.T_in) annotation (Line(
        points={{283.2,-93.02},{275.6,-93.02},{275.6,-92},{266,-92}}, color={0,
          0,127}));
  connect(chi1.port_a1, chi2.port_b1)
    annotation (Line(points={{4,-32},{-16,-32},{-16,-22}}, color={0,127,255}));
  connect(chi1.port_b2, chi2.port_a2) annotation (Line(points={{16,-32},{16,-26},
          {-4,-26},{-4,-22}}, color={0,127,255}));
  connect(chi2.port_b2, Sensor_pCHe.port)
    annotation (Line(points={{-4,-2},{-4,20},{26,20}}, color={0,127,255}));
  connect(Sensor_pCHe.port, junSu.port_1)
    annotation (Line(points={{26,20},{26,14},{72,14},{72,16},{78,16}},
                                               color={0,127,255}));
  connect(chi2.port_b2, Sensor_TCHe.port)
    annotation (Line(points={{-4,-2},{-4,20},{2,20}}, color={0,127,255}));
  connect(greaterThreshold.y, chi2.on) annotation (Line(points={{-65,38},{-12,
          38},{-12,6},{-13,6},{-13,0}}, color={255,0,255}));
  connect(SP_TCHe[2].y, chi2.TSet)
    annotation (Line(points={{-65,62},{-7,62},{-7,0}}, color={0,0,127}));
  connect(SP_TCHe[1].y, chi1.TSet)
    annotation (Line(points={{-65,62},{13,62},{13,-30}}, color={0,0,127}));
  connect(res1.port_b, expVesChi.port_a) annotation (Line(points={{-8,-106},{
          -40,-106},{-40,-107}}, color={0,127,255}));
  connect(cooTow.port_b, pumCW.port_a)
    annotation (Line(points={{-212,14},{-174,14}}, color={0,127,255}));
  connect(Twb.y, cooTow.TAir)
    annotation (Line(points={{-335,26},{-275.8,26}}, color={0,0,127}));
  connect(pumCW.port_b, Sensor_mCW.port_a)
    annotation (Line(points={{-154,14},{-106,14}}, color={0,127,255}));
  connect(Sensor_mCW.port_b, chi2.port_a1)
    annotation (Line(points={{-86,14},{-16,14},{-16,-2}}, color={0,127,255}));
  connect(expVesChi.port_a, cooTow.port_a) annotation (Line(points={{-40,-107},
          {-188,-107},{-188,-108},{-324,-108},{-324,14},{-270,14}}, color={0,
          127,255}));
  connect(Sensor_mCW.port_b, Sensor_TCWS.port)
    annotation (Line(points={{-86,14},{-46,14}}, color={0,127,255}));
  connect(expVesChi.port_a, Sensor_TCWR.port) annotation (Line(points={{-40,
          -107},{-99,-107},{-99,-106},{-158,-106}}, color={0,127,255}));
  connect(SP_mCH.y, SP_mCWS.u) annotation (Line(points={{91,-136},{-124,-136},{
          -124,-48},{-142,-48}}, color={0,0,127}));
  connect(SP_mCWS.y, pumCW.m_flow_in) annotation (Line(points={{-165,-48},{-166,
          -48},{-166,2},{-164,2}}, color={0,0,127}));
  connect(PID.y, cooTow.y) annotation (Line(points={{-315,66},{-302,66},{-302,
          38},{-275.8,38}}, color={0,0,127}));
  connect(SP_TCWS.y, PID.u_m)
    annotation (Line(points={{-375,46},{-326,46},{-326,54}}, color={0,0,127}));
  connect(Sensor_TCWS.T, PID.u_s) annotation (Line(points={{-39,24},{-36,24},{
          -36,86},{-346,86},{-346,66},{-338,66}}, color={0,0,127}));
  connect(res2.port_b, bou.ports[1]) annotation (Line(points={{250,20},{264,20},
          {264,36},{296,36},{296,18},{290,18}}, color={0,127,255}));
  connect(Sensor_mS.port_b, tan.port_a) annotation (Line(points={{94,-44},{94,
          -42},{122,-42},{122,-16},{126,-16},{126,-14},{145,-14},{145,-26}},
        color={0,127,255}));
  connect(tan.port_b, res3.port_a) annotation (Line(points={{145,-52},{145,-64},
          {180,-64},{180,-78},{212,-78},{212,-26},{170,-26},{170,-32}}, color={
          0,127,255}));
  connect(tan.port_a, Sensor_Ttankin.port) annotation (Line(points={{145,-26},{
          145,-16},{158,-16}}, color={0,127,255}));
  connect(tan.port_b, Sensor_Ttankout.port) annotation (Line(points={{145,-52},
          {145,-74},{128,-74}}, color={0,127,255}));
  connect(Sensor_psub_Suction.port, Pump_CHW_Secondary.port_a) annotation (Line(
        points={{140,20},{140,14},{166,14},{166,20},{172,20}}, color={0,127,255}));
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
    experiment(StopTime=300000, __Dymola_Algorithm="Dassl"));
end Chiller_Storage_CoolingTower_Original;
