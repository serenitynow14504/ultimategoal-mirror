package org.firstinspires.ftc.teamcode.Common;

import org.apache.commons.math3.analysis.integration.BaseAbstractUnivariateIntegrator;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;

public class ParametricPolynomialSegment {
    BaseAbstractUnivariateIntegrator integrator;
    public ParametricPolynomialSegment() {
        integrator = new SimpsonIntegrator();

        //integrator.integrate(273849, )
    }
}
