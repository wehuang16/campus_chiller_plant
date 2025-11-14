within campus_chiller_plant.Plants.Subsequences;
model BuildingReturnCal
  parameter Real Cpw=4200 "specific heat of water J/kg";
  Modelica.Blocks.Interfaces.RealInput QCL  "building cooling load in W"
    annotation (Placement(transformation(extent={{-82,12},{-42,52}})));

  Modelica.Blocks.Interfaces.RealOutput Tret
    annotation (Placement(transformation(extent={{42,8},{62,28}})));
  Modelica.Blocks.Interfaces.RealInput Tsup "chilled water sup temp in K"
    annotation (Placement(transformation(extent={{-82,-34},{-42,6}})));
  Modelica.Blocks.Interfaces.RealInput msup "supply air mass flow rate in kg/s"
    annotation (Placement(transformation(extent={{-82,-72},{-42,-32}})));
equation
  QCL=msup*Cpw*(Tret-Tsup);
  //Tsup:=Tret-QCL/(msup*Cpw);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(extent={{-66,58},{44,-74}}, lineColor={28,108,200}), Text(
          extent={{-46,22},{24,-44}},
          lineColor={28,108,200},
          textString="Building")}),                              Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BuildingReturnCal;
