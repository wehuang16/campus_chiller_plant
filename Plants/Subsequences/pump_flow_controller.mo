within campus_chiller_plant.Plants.Subsequences;
model pump_flow_controller

  parameter Modelica.Units.SI.MassFlowRate mEva1_flow_nominal=50
    "Nominal mass flow rate for chiller 1 evaporator side pump";
      parameter Modelica.Units.SI.MassFlowRate mEva2_flow_nominal=50
    "Nominal mass flow rate for chiller 2 evaporator side pump";
      parameter Modelica.Units.SI.MassFlowRate mCon1_flow_nominal=75
    "Nominal mass flow rate for chiller 1 condenser side pump";
      parameter Modelica.Units.SI.MassFlowRate mCon2_flow_nominal=75
    "Nominal mass flow rate for chiller 2 condenser side pump";

      parameter Modelica.Units.SI.MassFlowRate mBypass_flow_nominal=10
    "Nominal mass flow rate for bypass pump";
       parameter Modelica.Units.SI.MassFlowRate mTankDischarge_flow_nominal=50
    "Nominal mass flow rate for bypass pump";
  Buildings.Controls.OBC.CDL.Interfaces.IntegerInput
                                                   systemMode
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,24})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput chiller1On
    annotation (Placement(transformation(extent={{238,60},{278,100}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput chiller2On
    annotation (Placement(transformation(extent={{238,12},{278,52}})));
  Buildings.Controls.OBC.CDL.Integers.Equal chiller1OnBlock
    annotation (Placement(transformation(extent={{-32,64},{-12,84}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.Constant conInt(k=1)
    annotation (Placement(transformation(extent={{-74,110},{-54,130}})));
  Buildings.Controls.OBC.CDL.Integers.Equal chiller2OnBlock
    annotation (Placement(transformation(extent={{-32,36},{-12,56}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.Constant conInt1(k=2)
    annotation (Placement(transformation(extent={{-88,60},{-68,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput mEva1_flow
    annotation (Placement(transformation(extent={{238,-24},{278,16}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput mEva2_flow
    annotation (Placement(transformation(extent={{238,-56},{278,-16}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput mCon1_flow
    annotation (Placement(transformation(extent={{238,-92},{278,-52}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput mCon2_flow
    annotation (Placement(transformation(extent={{238,-124},{278,-84}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea(realTrue=
        mEva1_flow_nominal)
    annotation (Placement(transformation(extent={{4,-10},{24,10}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea1(realTrue=
        mEva2_flow_nominal)
    annotation (Placement(transformation(extent={{184,-48},{204,-28}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea2(realTrue=
        mCon1_flow_nominal)
    annotation (Placement(transformation(extent={{12,-78},{32,-58}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea3(realTrue=
        mCon2_flow_nominal)
    annotation (Placement(transformation(extent={{178,-116},{198,-96}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput mBypass_flow
    annotation (Placement(transformation(extent={{238,-162},{278,-122}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput mSecondary_flow
    annotation (Placement(transformation(extent={{240,-210},{280,-170}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput hx_water_pid annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-118,-158})));
  Buildings.Controls.OBC.CDL.Reals.Multiply mul
    annotation (Placement(transformation(extent={{76,-22},{96,-2}})));
  Buildings.Controls.OBC.CDL.Reals.Multiply mul1
    annotation (Placement(transformation(extent={{72,-110},{92,-90}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea4(realTrue=
        mBypass_flow_nominal)
    annotation (Placement(transformation(extent={{188,-152},{208,-132}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea5(realTrue=
        mTankDischarge_flow_nominal)
    annotation (Placement(transformation(extent={{48,-210},{68,-190}})));
  Buildings.Controls.OBC.CDL.Integers.Equal chiller2OnBlock1
    annotation (Placement(transformation(extent={{-42,-22},{-22,-2}})));
  Buildings.Controls.OBC.CDL.Integers.Sources.Constant conInt2(k=3)
    annotation (Placement(transformation(extent={{-76,8},{-56,28}})));
  Buildings.Controls.OBC.CDL.Reals.Add add2
    annotation (Placement(transformation(extent={{168,-206},{188,-186}})));
  Buildings.Controls.OBC.CDL.Reals.Multiply mul2
    annotation (Placement(transformation(extent={{100,-228},{120,-208}})));
equation
  connect(systemMode, chiller1OnBlock.u1) annotation (Line(points={{-120,24},{
          -82,24},{-82,42},{-46,42},{-46,74},{-34,74}},
                                 color={255,127,0}));
  connect(conInt.y, chiller1OnBlock.u2) annotation (Line(points={{-52,120},{-44,
          120},{-44,76},{-42,76},{-42,66},{-34,66}},
                              color={255,127,0}));
  connect(systemMode, chiller2OnBlock.u1) annotation (Line(points={{-120,24},{
          -82,24},{-82,42},{-46,42},{-46,46},{-34,46}},
                                 color={255,127,0}));
  connect(conInt1.y, chiller2OnBlock.u2) annotation (Line(points={{-66,70},{-48,
          70},{-48,38},{-34,38}}, color={255,127,0}));
  connect(chiller1OnBlock.y, chiller1On) annotation (Line(points={{-10,74},{232,
          74},{232,80},{258,80}},
                             color={255,0,255}));
  connect(chiller2OnBlock.y, chiller2On) annotation (Line(points={{-10,46},{94,46},
          {94,32},{258,32}}, color={255,0,255}));
  connect(chiller1OnBlock.y, booToRea.u) annotation (Line(points={{-10,74},{58,74},
          {58,6},{34,6},{34,16},{-8,16},{-8,0},{2,0}}, color={255,0,255}));
  connect(booToRea1.y, mEva2_flow) annotation (Line(points={{206,-38},{232,-38},
          {232,-36},{258,-36}}, color={0,0,127}));
  connect(booToRea3.y, mCon2_flow) annotation (Line(points={{200,-106},{232,-106},
          {232,-104},{258,-104}}, color={0,0,127}));
  connect(chiller1OnBlock.y, booToRea2.u) annotation (Line(points={{-10,74},{58,
          74},{58,6},{34,6},{34,16},{-8,16},{-8,-68},{10,-68}}, color={255,0,255}));
  connect(chiller2OnBlock.y, booToRea1.u) annotation (Line(points={{-10,46},{48,
          46},{48,-38},{182,-38}}, color={255,0,255}));
  connect(chiller2OnBlock.y, booToRea3.u) annotation (Line(points={{-10,46},{48,
          46},{48,-38},{174,-38},{174,-90},{168,-90},{168,-106},{176,-106}},
        color={255,0,255}));
  connect(booToRea.y, mul.u1)
    annotation (Line(points={{26,0},{64,0},{64,-6},{74,-6}}, color={0,0,127}));
  connect(mul.y, mEva1_flow) annotation (Line(points={{98,-12},{232,-12},{232,-4},
          {258,-4}}, color={0,0,127}));
  connect(booToRea2.y, mul1.u1) annotation (Line(points={{34,-68},{60,-68},{60,-94},
          {70,-94}}, color={0,0,127}));
  connect(mul1.y, mCon1_flow) annotation (Line(points={{94,-100},{166,-100},{166,
          -88},{232,-88},{232,-72},{258,-72}}, color={0,0,127}));
  connect(booToRea4.y, mBypass_flow)
    annotation (Line(points={{210,-142},{258,-142}}, color={0,0,127}));
  connect(chiller2OnBlock.y, booToRea4.u) annotation (Line(points={{-10,46},{-16,
          46},{-16,-162},{186,-162},{186,-142}}, color={255,0,255}));
  connect(conInt2.y, chiller2OnBlock1.u2) annotation (Line(points={{-54,18},{
          -46,18},{-46,4},{-48,4},{-48,-4},{-52,-4},{-52,-20},{-44,-20}}, color
        ={255,127,0}));
  connect(systemMode, chiller2OnBlock1.u1) annotation (Line(points={{-120,24},{
          -82,24},{-82,-12},{-44,-12}},color={255,127,0}));
  connect(chiller2OnBlock1.y, booToRea5.u) annotation (Line(points={{-20,-12},{
          -24,-12},{-24,-192},{46,-192},{46,-200}}, color={255,0,255}));
  connect(add2.y, mSecondary_flow) annotation (Line(points={{190,-196},{234,
          -196},{234,-190},{260,-190}}, color={0,0,127}));
  connect(booToRea5.y, mul2.u1) annotation (Line(points={{70,-200},{90,-200},{
          90,-212},{98,-212}}, color={0,0,127}));
  connect(mul2.y, add2.u2) annotation (Line(points={{122,-218},{156,-218},{156,
          -202},{166,-202}}, color={0,0,127}));
  connect(mul.y, add2.u1) annotation (Line(points={{98,-12},{164,-12},{164,-180},
          {158,-180},{158,-190},{166,-190}}, color={0,0,127}));
  connect(hx_water_pid, mul.u2) annotation (Line(points={{-118,-158},{60,-158},
          {60,-96},{62,-96},{62,-18},{74,-18}}, color={0,0,127}));
  connect(hx_water_pid, mul1.u2) annotation (Line(points={{-118,-158},{60,-158},
          {60,-106},{70,-106}}, color={0,0,127}));
  connect(hx_water_pid, mul2.u2) annotation (Line(points={{-118,-158},{-26,-158},
          {-26,-224},{98,-224}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -250},{240,100}},
        grid={2,2})),                                            Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-250},{240,
            100}},
        grid={2,2})));
end pump_flow_controller;
