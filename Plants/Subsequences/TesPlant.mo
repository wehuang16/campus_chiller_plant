within campus_chiller_plant.Plants.Subsequences;
model TesPlant

          package MediumAir = Buildings.Media.Air;
  package MediumWater = Buildings.Media.Water;
    package MediumPropyleneGlycol =
      Buildings.Media.Antifreeze.PropyleneGlycolWater (property_T=273.15+50, X_a=
            0.4);

  parameter Modelica.Units.SI.MassFlowRate  m_flow_nominal=3;
  Buildings.Fluid.Actuators.Valves.TwoWayLinear val4(
    redeclare package Medium = MediumWater,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=10) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={98,62})));
  Buildings.Fluid.Movers.SpeedControlled_y mov2(
    redeclare package Medium = MediumWater,
    redeclare Buildings.Fluid.Movers.Data.Pumps.Wilo.Stratos30slash1to8 per,
    addPowerToMedium=false)
    annotation (Placement(transformation(extent={{4,-34},{24,-14}})));
  Buildings.Fluid.Storage.StratifiedEnhanced tan(
    redeclare package Medium = MediumWater,
    m_flow_nominal=m_flow_nominal,
    VTan=100,
    hTan=5,
    dIns=1,
    nSeg=20)
    annotation (Placement(transformation(extent={{-168,4},{-148,24}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant conRea1(k=1)
    "Real inputs"
    annotation (Placement(transformation(extent={{-176,132},{-156,152}})));
  Buildings.Fluid.Actuators.Valves.TwoWayLinear val8(
    redeclare package Medium = MediumWater,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=10)
    annotation (Placement(transformation(extent={{-126,26},{-106,46}})));
  Buildings.Fluid.Actuators.Valves.TwoWayLinear val7(
    redeclare package Medium = MediumWater,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=10)
    annotation (Placement(transformation(extent={{-110,-46},{-134,-22}})));
  Buildings.Fluid.Actuators.Valves.TwoWayLinear val6(
    redeclare package Medium = MediumWater,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=10)
    annotation (Placement(transformation(extent={{-10,68},{10,88}})));
  Buildings.Fluid.Actuators.Valves.TwoWayLinear val3(
    redeclare package Medium = MediumWater,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=10)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-48,18})));
  Buildings.Fluid.FixedResistances.Junction junConv(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{94,96},{114,116}})));
  Buildings.Fluid.FixedResistances.Junction junDiv(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0})
    annotation (Placement(transformation(extent={{-68,66},{-48,86}})));
  Buildings.Fluid.FixedResistances.Junction junDiv1(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=90,
        origin={-48,-26})));
  Buildings.Fluid.FixedResistances.Junction junDiv2(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=90,
        origin={-48,-78})));
  Buildings.Fluid.FixedResistances.Junction junDiv3(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=90,
        origin={-50,-128})));
  Buildings.Fluid.Movers.SpeedControlled_y mov1(
    redeclare package Medium = MediumWater,
    redeclare Buildings.Fluid.Movers.Data.Pumps.Wilo.Stratos30slash1to8 per,
    addPowerToMedium=false)
    annotation (Placement(transformation(extent={{6,-88},{26,-68}})));
  Buildings.Fluid.Movers.SpeedControlled_y mov3(
    redeclare package Medium = MediumWater,
    redeclare Buildings.Fluid.Movers.Data.Pumps.Wilo.Stratos30slash1to8 per,
    addPowerToMedium=false)
    annotation (Placement(transformation(extent={{6,-140},{26,-120}})));
  Buildings.Fluid.Actuators.Valves.TwoWayLinear val1(
    redeclare package Medium = MediumWater,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=10)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-52,-180})));
  Buildings.Fluid.FixedResistances.Junction junConv1(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-52,-224})));
  Buildings.Fluid.Actuators.Valves.TwoWayLinear val5(
    redeclare package Medium = MediumWater,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=10)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={12,-208})));
  Buildings.Fluid.Actuators.Valves.TwoWayLinear val2(
    redeclare package Medium = MediumWater,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=10)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={96,-164})));
  Buildings.Fluid.FixedResistances.Junction junDiv4(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{13,13},{-13,-13}},
        rotation=0,
        origin={97,-217})));
  Buildings.Fluid.FixedResistances.Junction junConv2(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{15,-15},{-15,15}},
        rotation=270,
        origin={93,-119})));
  Buildings.Fluid.FixedResistances.Junction junConv3(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{15,-15},{-15,15}},
        rotation=270,
        origin={97,-69})));
  Buildings.Fluid.FixedResistances.Junction junConv4(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={0,0,0}) annotation (Placement(transformation(
        extent={{15,-15},{-15,15}},
        rotation=270,
        origin={95,-19})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanInput chargeMode
    "true=charging, false=discharging"
    annotation (Placement(transformation(extent={{-240,40},{-200,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput pump_speed
    annotation (Placement(transformation(extent={{-240,-124},{-200,-84}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea
    annotation (Placement(transformation(extent={{-166,86},{-146,106}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea1(realTrue=0,
      realFalse=1)
    annotation (Placement(transformation(extent={{-212,22},{-192,42}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
        MediumWater)
    annotation (Placement(transformation(extent={{178,-238},{198,-218}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
        MediumWater)
    annotation (Placement(transformation(extent={{176,74},{196,94}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensorTop
    annotation (Placement(transformation(extent={{48,26},{68,46}})));
  Buildings.Controls.OBC.CDL.Interfaces.IntegerOutput tesStatus
    "1=fully charged; 0=has capacity; -1=fully discharged"
                                         annotation (Placement(transformation(
          extent={{180,-98},{220,-58}}), iconTransformation(extent={{180,-98},{
            220,-58}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensorBottom
    annotation (Placement(transformation(extent={{48,-10},{68,10}})));
  TesStatusController tesStatusController
    annotation (Placement(transformation(extent={{180,-2},{200,18}})));
equation
  connect(val7.port_b, tan.port_b) annotation (Line(points={{-134,-34},{-158,-34},
          {-158,4}}, color={0,127,255}));
  connect(val8.port_a, tan.port_a) annotation (Line(points={{-126,36},{-144,36},
          {-144,40},{-158,40},{-158,24}}, color={0,127,255}));
  connect(val8.port_b, junDiv.port_1) annotation (Line(points={{-106,36},{-96,
          36},{-96,38},{-82,38},{-82,76},{-68,76}}, color={0,127,255}));
  connect(junDiv.port_2, val6.port_a) annotation (Line(points={{-48,76},{-16,76},
          {-16,78},{-10,78}}, color={0,127,255}));
  connect(junDiv.port_3, val3.port_a) annotation (Line(points={{-58,66},{-58,34},
          {-48,34},{-48,28}}, color={0,127,255}));
  connect(val3.port_b, junDiv1.port_1)
    annotation (Line(points={{-48,8},{-48,-14}}, color={0,127,255}));
  connect(junDiv1.port_3, mov2.port_a) annotation (Line(points={{-36,-26},{-2,
          -26},{-2,-24},{4,-24}}, color={0,127,255}));
  connect(junDiv2.port_3, mov1.port_a)
    annotation (Line(points={{-36,-78},{6,-78}}, color={0,127,255}));
  connect(junDiv3.port_3, mov3.port_a) annotation (Line(points={{-38,-128},{0,
          -128},{0,-130},{6,-130}}, color={0,127,255}));
  connect(junDiv1.port_2, junDiv2.port_1)
    annotation (Line(points={{-48,-38},{-48,-66}}, color={0,127,255}));
  connect(junDiv2.port_2, junDiv3.port_1) annotation (Line(points={{-48,-90},{
          -48,-110},{-50,-110},{-50,-116}}, color={0,127,255}));
  connect(junDiv3.port_2, val1.port_a) annotation (Line(points={{-50,-140},{-50,
          -164},{-52,-164},{-52,-170}}, color={0,127,255}));
  connect(val1.port_b, junConv1.port_3)
    annotation (Line(points={{-52,-190},{-52,-214}}, color={0,127,255}));
  connect(val7.port_a, junConv1.port_2) annotation (Line(points={{-110,-34},{
          -96,-34},{-96,-224},{-62,-224}}, color={0,127,255}));
  connect(val5.port_b, junConv1.port_1) annotation (Line(points={{2,-208},{-36,
          -208},{-36,-224},{-42,-224}}, color={0,127,255}));
  connect(junDiv4.port_2, val5.port_a) annotation (Line(points={{84,-217},{84,
          -218},{28,-218},{28,-208},{22,-208}}, color={0,127,255}));
  connect(junDiv4.port_3, val2.port_a) annotation (Line(points={{97,-204},{97,
          -180},{96,-180},{96,-174}}, color={0,127,255}));
  connect(mov3.port_b, junConv2.port_3) annotation (Line(points={{26,-130},{50,
          -130},{50,-122},{78,-122},{78,-119}}, color={0,127,255}));
  connect(val2.port_b, junConv2.port_1) annotation (Line(points={{96,-154},{96,
          -140},{93,-140},{93,-134}}, color={0,127,255}));
  connect(junConv2.port_2, junConv3.port_1) annotation (Line(points={{93,-104},
          {93,-90},{97,-90},{97,-84}}, color={0,127,255}));
  connect(mov1.port_b, junConv3.port_3) annotation (Line(points={{26,-78},{76,
          -78},{76,-69},{82,-69}}, color={0,127,255}));
  connect(junConv4.port_1, junConv3.port_2) annotation (Line(points={{95,-34},{
          96,-34},{96,-54},{97,-54}}, color={0,127,255}));
  connect(mov2.port_b, junConv4.port_3) annotation (Line(points={{24,-24},{74,
          -24},{74,-19},{80,-19}}, color={0,127,255}));
  connect(junConv4.port_2, val4.port_a) annotation (Line(points={{95,-4},{95,46},
          {98,46},{98,52}}, color={0,127,255}));
  connect(val6.port_b, junConv.port_1) annotation (Line(points={{10,78},{88,78},
          {88,106},{94,106}}, color={0,127,255}));
  connect(junConv.port_3, val4.port_b) annotation (Line(points={{104,96},{104,
          78},{98,78},{98,72}}, color={0,127,255}));
  connect(pump_speed, mov2.y) annotation (Line(points={{-220,-104},{0,-104},{0,
          -58},{30,-58},{30,-4},{14,-4},{14,-12}}, color={0,0,127}));
  connect(pump_speed, mov1.y) annotation (Line(points={{-220,-104},{0,-104},{0,
          -58},{16,-58},{16,-66}}, color={0,0,127}));
  connect(pump_speed, mov3.y) annotation (Line(points={{-220,-104},{16,-104},{
          16,-118}}, color={0,0,127}));
  connect(chargeMode, booToRea.u) annotation (Line(points={{-220,60},{-178,60},{
          -178,96},{-168,96}}, color={255,0,255}));
  connect(chargeMode, booToRea1.u) annotation (Line(points={{-220,60},{-220,32},
          {-214,32}}, color={255,0,255}));
  connect(conRea1.y, val8.y) annotation (Line(points={{-154,142},{-116,142},{
          -116,48}}, color={0,0,127}));
  connect(conRea1.y, val7.y) annotation (Line(points={{-154,142},{-154,-19.6},{
          -122,-19.6}}, color={0,0,127}));
  connect(booToRea.y, val3.y)
    annotation (Line(points={{-144,96},{-36,96},{-36,18}}, color={0,0,127}));
  connect(booToRea.y, val4.y) annotation (Line(points={{-144,96},{-36,96},{-36,
          62},{86,62}}, color={0,0,127}));
  connect(booToRea1.y, val1.y) annotation (Line(points={{-190,32},{-186,32},{
          -186,0},{-146,0},{-146,-200},{-40,-200},{-40,-180}}, color={0,0,127}));
  connect(booToRea1.y, val2.y) annotation (Line(points={{-190,32},{-146,32},{
          -146,-200},{84,-200},{84,-164}}, color={0,0,127}));
  connect(booToRea.y, val5.y) annotation (Line(points={{-144,96},{-118,96},{
          -118,90},{-80,90},{-80,-272},{12,-272},{12,-220}}, color={0,0,127}));
  connect(booToRea1.y, val6.y) annotation (Line(points={{-190,32},{-92,32},{-92,
          108},{0,108},{0,90}}, color={0,0,127}));
  connect(junConv.port_2, port_b) annotation (Line(points={{114,106},{170,106},
          {170,84},{186,84}}, color={0,127,255}));
  connect(junDiv4.port_1, port_a) annotation (Line(points={{110,-217},{110,-218},
          {180,-218},{180,-228},{188,-228}}, color={0,127,255}));
  connect(temperatureSensorTop.port, tan.heaPorVol[3]) annotation (Line(points=
          {{48,36},{-90,36},{-90,13.775},{-158,13.775}}, color={191,0,0}));
  connect(temperatureSensorBottom.port, tan.heaPorVol[17]) annotation (Line(
        points={{48,0},{-28,0},{-28,36},{-90,36},{-90,14.195},{-158,14.195}},
        color={191,0,0}));
  connect(temperatureSensorTop.T, tesStatusController.TesTopTemp) annotation (
      Line(points={{69,36},{168,36},{168,11.8},{177.8,11.8}}, color={0,0,127}));
  connect(temperatureSensorBottom.T, tesStatusController.TesBottomTemp)
    annotation (Line(points={{69,0},{74,0},{74,1.8},{178,1.8}}, color={0,0,127}));
  connect(tesStatusController.TesMode, tesStatus) annotation (Line(points={{202,
          8},{210,8},{210,-52},{174,-52},{174,-78},{200,-78}}, color={255,127,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,
            -260},{180,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-200,-260},{180,
            100}})),
    experiment(
      StopTime=86400,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end TesPlant;
