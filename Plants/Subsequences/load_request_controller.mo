within campus_chiller_plant.Plants.Subsequences;
model load_request_controller
  parameter Modelica.Units.SI.Temperature oat_lockout_temperature=273.15+20
    "oat_lockout_temperature";

  Buildings.Controls.OBC.CDL.Reals.Hysteresis OAT_lockout(uLow=
        oat_lockout_temperature - 0.1, uHigh=oat_lockout_temperature)
    annotation (Placement(transformation(extent={{-82,46},{-62,66}})));
  Buildings.Controls.OBC.CDL.Logical.And and2
    annotation (Placement(transformation(extent={{30,-4},{50,16}})));
  Buildings.Controls.OBC.CDL.Reals.Hysteresis hys(uLow=0.05, uHigh=0.1)
    annotation (Placement(transformation(extent={{-44,-68},{-24,-48}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput hx_water_pid annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-116,-56})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput outside_air_temperature
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.BooleanOutput loadRequest
    "true=need cooling; false=deadband"
    annotation (Placement(transformation(extent={{100,-20},{140,20}})));
equation

  connect(OAT_lockout.y,and2. u1) annotation (Line(points={{-60,56},{18,56},{18,
          6},{28,6}},                                         color={255,0,255}));
  connect(hys.y,and2. u2) annotation (Line(points={{-22,-58},{20,-58},{20,-2},{
          28,-2}},              color={255,0,255}));
  connect(hx_water_pid, hys.u) annotation (Line(points={{-116,-56},{-54,-56},{
          -54,-58},{-46,-58}}, color={0,0,127}));
  connect(OAT_lockout.u, outside_air_temperature) annotation (Line(points={{-84,
          56},{-92,56},{-92,60},{-120,60}}, color={0,0,127}));
  connect(and2.y, loadRequest) annotation (Line(points={{52,6},{94,6},{94,0},{
          120,0}}, color={255,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end load_request_controller;
