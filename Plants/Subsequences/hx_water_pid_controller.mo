within campus_chiller_plant.Plants.Subsequences;
model hx_water_pid_controller
  Buildings.Controls.OBC.CDL.Interfaces.RealInput hx_water_temperature_setpoint
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,54})));
  Buildings.Controls.Continuous.LimPID conPID(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.3,
    Ti=150,
    initType=Modelica.Blocks.Types.Init.InitialOutput,
    y_start=0.2,
    reverseActing=false)
    annotation (Placement(transformation(extent={{0,4},{20,24}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput hx_water_temperature
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-48})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput hx_water_pid_out annotation
    (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={120,0})));
equation
  connect(hx_water_temperature_setpoint,conPID. u_s) annotation (Line(points={{-120,54},
          {-12,54},{-12,14},{-2,14}},          color={0,0,127}));
  connect(hx_water_temperature,conPID. u_m) annotation (Line(points={{-120,-48},
          {10,-48},{10,2}},     color={0,0,127}));
  connect(conPID.y, hx_water_pid_out) annotation (Line(points={{21,14},{94,14},
          {94,0},{120,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end hx_water_pid_controller;
