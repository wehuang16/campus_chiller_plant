within campus_chiller_plant.Examples.BaseClasses;
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
  Buildings.Fluid.Sources.Boundary_pT bou1(redeclare package Medium =
        MediumWater, nPorts=1)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={200,-210})));
  Buildings.Fluid.Sources.Boundary_pT bou2(redeclare package Medium =
        MediumWater, nPorts=1)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={196,100})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant conRea4(k=0.6)
    "Real inputs"
    annotation (Placement(transformation(extent={{4,28},{24,48}})));
  Buildings.Controls.OBC.CDL.Reals.Sources.Constant conRea1(k=0.6)
    "Real inputs"
    annotation (Placement(transformation(extent={{38,134},{58,154}})));
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
    dp_nominal={1,-1,1})
    annotation (Placement(transformation(extent={{94,96},{114,116}})));
  Buildings.Fluid.FixedResistances.Junction junDiv(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={1,-1,-1})
    annotation (Placement(transformation(extent={{-68,66},{-48,86}})));
  Buildings.Fluid.FixedResistances.Junction junDiv1(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={1,-1,-1}) annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=90,
        origin={-48,-26})));
  Buildings.Fluid.FixedResistances.Junction junDiv2(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={1,-1,-1}) annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=90,
        origin={-48,-78})));
  Buildings.Fluid.FixedResistances.Junction junDiv3(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,-1},
    dp_nominal={1,-1,-1}) annotation (Placement(transformation(
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
    dp_nominal={1,-1,1}) annotation (Placement(transformation(
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
    dp_nominal={1,-1,-1}) annotation (Placement(transformation(
        extent={{13,13},{-13,-13}},
        rotation=0,
        origin={97,-217})));
  Buildings.Fluid.FixedResistances.Junction junConv2(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={1,-1,1}) annotation (Placement(transformation(
        extent={{15,-15},{-15,15}},
        rotation=270,
        origin={93,-119})));
  Buildings.Fluid.FixedResistances.Junction junConv3(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={1,-1,1}) annotation (Placement(transformation(
        extent={{15,-15},{-15,15}},
        rotation=270,
        origin={97,-69})));
  Buildings.Fluid.FixedResistances.Junction junConv4(
    redeclare package Medium = MediumWater,
    m_flow_nominal={1,-1,1},
    dp_nominal={1,-1,1}) annotation (Placement(transformation(
        extent={{15,-15},{-15,15}},
        rotation=270,
        origin={95,-19})));
equation
  connect(conRea4.y, mov2.y) annotation (Line(points={{26,38},{34,38},{34,-2},{
          14,-2},{14,-12}},
                color={0,0,127}));
  connect(conRea1.y, val4.y)
    annotation (Line(points={{60,144},{86,144},{86,62}}, color={0,0,127}));
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
  connect(bou1.ports[1], junDiv4.port_1) annotation (Line(points={{190,-210},{
          150,-210},{150,-217},{110,-217}}, color={0,127,255}));
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
  connect(junConv.port_2, bou2.ports[1]) annotation (Line(points={{114,106},{
          132,106},{132,100},{186,100}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=86400,
      Interval=60,
      __Dymola_Algorithm="Dassl"));
end TesPlant;
