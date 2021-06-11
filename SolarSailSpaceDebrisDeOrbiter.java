package org.orekit.tutorials.own;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.*;
import java.collections.*;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.RotationOrder;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.ode.nonstiff.AdaptiveStepsizeIntegrator;
import org.hipparchus.ode.nonstiff.DormandPrince853Integrator;
import org.hipparchus.util.FastMath;
import org.orekit.attitudes.Attitude;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.AttitudesSequence;
import org.orekit.attitudes.CelestialBodyPointed;
import org.orekit.attitudes.LofOffset;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.forces.BoxAndSolarArraySpacecraft;
import org.orekit.forces.ForceModel;
import org.orekit.forces.radiation.IsotropicRadiationSingleCoefficient;
import org.orekit.forces.radiation.RadiationSensitive;
import org.orekit.forces.radiation.SolarRadiationPressure;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.LOFType;
import org.orekit.frames.Transform;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.EcksteinHechlerPropagator;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.events.AlignmentDetector;
import org.orekit.propagation.events.AltitudeDetector;
import org.orekit.propagation.events.EclipseDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.events.handlers.ContinueOnEvent;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.DateComponents;
import org.orekit.time.TimeComponents;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.time.UTCScale;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.ExtendedPVCoordinatesProvider;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.PVCoordinatesProvider;
import org.orekit.utils.ParameterDriver;
import org.orekit.utils.TimeStampedAngularCoordinates;

public class SolarSailSpaceDebrisDeOrbiter {

    /** Private constructor for utility class. */
    private SolarSailSpaceDebrisDeOrbiter() {
        // empty
    }

     
    /** Program entry point.
     * @param args program arguments (unused here)
     */
    public static void main(final String[] args) {
    	
        try {

            // configure Orekit
            final File home       = new File(System.getProperty("user.home"));
            final File orekitData = new File(home, "orekit-data");
            if (!orekitData.exists()) {
                System.err.format(Locale.US, "Failed to find %s folder%n",
                                  orekitData.getAbsolutePath());
                System.err.format(Locale.US, "You need to download %s from %s, unzip it in %s and rename it 'orekit-data' for this tutorial to work%n",
                                  "orekit-data-master.zip", "https://gitlab.orekit.org/orekit/orekit-data/-/archive/master/orekit-data-master.zip",
                                  home.getAbsolutePath());
                System.exit(1);
            }
            final DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
            manager.addProvider(new DirectoryCrawler(orekitData));

//            final SortedSet<String> output = new TreeSet<>();

            //  Initial state definition : date, orbit
            final AbsoluteDate initialDate =  new AbsoluteDate(new DateComponents(2021, 2, 3), new TimeComponents(0, 3, 0),TimeScalesFactory.getUTC());
            final Frame eme2000 = FramesFactory.getEME2000();
            // for equinocital orbit: semi major axis(a), first component of eccentricity vector (ex), second component of eccentricity vector (ey)
            //, first component of inclination vector (hx), second component of inclination vector(hy), mean/eccentic/true longitude arguement (l)
            
            final double e = 0.0;//eccentricity
            final double i = FastMath.toRadians(0.0); //inclination
            final double pa = FastMath.toRadians(0); //perigee arguement
            final double raan = FastMath.toRadians(0); //right ascension of ascending node
            final double v = FastMath.toRadians(0); // true anomaly
         
            final double a = 6928000; // altitude of 550 km 
            final double ex = e * FastMath.cos(pa + raan);
            final double ey = e * FastMath.sin(pa + raan);
            final double hx = FastMath.tan(i/2) * FastMath.cos(raan);
            final double hy = FastMath.tan(i/2) * FastMath.sin(raan);
            final double l = v + pa + raan;
            
            final Orbit initialOrbit =
            		new EquinoctialOrbit(a, ex, ey, hx, hy, l, PositionAngle.TRUE, eme2000, initialDate, Constants.EIGEN5C_EARTH_MU);
            
            
            
            final ExtendedPVCoordinatesProvider sun = CelestialBodyFactory.getSun();
            final OneAxisEllipsoid earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                    0.0,
                    FramesFactory.getITRF(IERSConventions.IERS_2010, true));
            
//           Spacecraft intial state defintion:
            
            final SpacecraftState initialState = new SpacecraftState(initialOrbit, 100.0); //orbit and mass in kg
            
            int sidelength = 100; //meters
            double absorptioncoeff = 0.1;
            double reflectioncoeff = 0.9;
            //length of 0 in x, and sidelength in y and z
            //0 array of solar arrays and arbitrary pointing vector for solar arrays
            final RadiationSensitive spacecraft = new BoxAndSolarArraySpacecraft(0, sidelength, sidelength, sun, 0, Vector3D.MINUS_K, 0, absorptioncoeff, reflectioncoeff); 
            
            
            //Attitude laws:
            
            final AttitudeProvider pointToSunLaw = new CelestialBodyPointed(initialOrbit.getFrame(), sun,
                    Vector3D.PLUS_K, Vector3D.PLUS_I, Vector3D.PLUS_K);
            
            final AttitudeProvider AwayFromSunLaw = new CelestialBodyPointed(initialOrbit.getFrame(), sun,
                    Vector3D.PLUS_K, Vector3D.PLUS_K, Vector3D.PLUS_I);
            
            final AttitudeProvider ToVelocityVectorLaw = new LofOffset(eme2000, LOFType.VNC);
                        
            
            
            //event detector to detect when spacecraft crosses the closest point to sun
            final EventDetector crossSun = new AlignmentDetector(1,initialOrbit, sun, 0).
                    withHandler(new ContinueOnEvent<AlignmentDetector>());
			
            
            //retrograde law that is a sequence of AwayFromSunLaw and ToVelocityVectorLaw
			final AttitudesSequence retrograde = new AttitudesSequence();
			final AttitudesSequence.SwitchHandler switchHandler =
				(preceding, following, s) -> {
					if (preceding == AwayFromSunLaw) {
//						output.add(s.getDate() + ": switching to toVelocityVectorLaw");
					} else {
//						output.add(s.getDate() + ": switching to AwayFromSunLaw");
					}
			};
			
			//add switching conditions to switch between two laws based on event function cross sun
			retrograde.addSwitchingCondition(AwayFromSunLaw, ToVelocityVectorLaw, crossSun,
			                        false, true, 10.0,
			                        AngularDerivativesFilter.USE_R, switchHandler);
			retrograde.addSwitchingCondition(ToVelocityVectorLaw, AwayFromSunLaw, crossSun,
			                        true, false, 10.0,
			                        AngularDerivativesFilter.USE_R, switchHandler);
			
			
			//intial law for retrograde
			if (crossSun.g(new SpacecraftState(initialOrbit)) >= 0) {
				// initial position is in daytime
				retrograde.resetActiveProvider(AwayFromSunLaw);
			} else {
				// initial position is in nighttime
				retrograde.resetActiveProvider(ToVelocityVectorLaw);
			}
            
            final double equtorialRadius = 6378000; //in meters
       
         // prepare numerical propagator
            final OrbitType orbitType = initialOrbit.getType();
            
            final ForceModel srpModel = new SolarRadiationPressure(sun, equtorialRadius, spacecraft);
            final AttitudeProvider SolarSail = new SolarSail(eme2000, LOFType.VNC, (SolarRadiationPressure) srpModel);
            
            //detects when altitude of 70 km has been reached, below karman line
            EventDetector karmanline = new AltitudeDetector( 70000 , earth);
            
            
            
            final double[][] tol = NumericalPropagator.tolerances(1.0, initialOrbit, orbitType);
            final AdaptiveStepsizeIntegrator integrator =
                            new DormandPrince853Integrator(0.001, 1000, tol[0], tol[1]);
            integrator.setInitialStepSize(60);
            final NumericalPropagator propagator = new NumericalPropagator(integrator);
            
            
            propagator.setOrbitType(orbitType);
            propagator.setInitialState(initialState);
            
//            
//            propagator.setAttitudeProvider(AwayFromSunLaw);
//            propagator.setAttitudeProvider(ToVelocityVectorLaw);
            propagator.setAttitudeProvider(retrograde);
//            propagator.setAttitudeProvider(pointToSunLaw);
            propagator.addForceModel(srpModel);
            
            // Register the switching events to the propagator
            retrograde.registerSwitchEvents(propagator);
            
            //stops when descedning below 70km
            propagator.addEventDetector(karmanline);
            
            ArrayList<ArrayList<String>> toFile = new ArrayList<ArrayList<String>>();
            	
            propagator.setMasterMode(60.0, (state, isLast) -> {
//            	final LOFType Lof = LOFType.VNC;
//            	
//                final Attitude attitude = state.getAttitude();
//                final Rotation OffSetAttRot = attitude.getRotation();
//                final Rotation AlligedAttRot = new LofOffset(eme2000, Lof).getAttitude(state.getOrbit(), state.getDate(), eme2000).getRotation();
//                final Rotation  offsetProper = OffSetAttRot.compose(AlligedAttRot.revert(), RotationConvention.VECTOR_OPERATOR);
//                
//                // note the call to revert and the conventions in the following statement
//                double[] anglesV = offsetProper.revert().getAngles(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR);
//                
//                
//                final ParameterDriver[] srpParamDrivs = srpModel.getParametersDrivers();
//                final int srpParamlen = srpParamDrivs.length;
//                final double [] srpParams = new double [srpParamlen];
//                int count = 0;
//                
//                for(ParameterDriver p: srpParamDrivs) {
//                	srpParams[count] = p.getValue();
//                	count++;
//                }
//                
//                final Vector3D srpAccVectorInert = srpModel.acceleration(state, srpParams);
//                
//                final Transform frameToinertial = state.getFrame().getTransformTo(eme2000, state.getDate());
//                final Transform inertialToFrame = eme2000.getTransformTo(state.getFrame(), state.getDate());
//                
//                
////                final Vector3D srpAccVector = inertialToFrame.transformVector(srpAccVectorInert);
//                
//                final AbsoluteDate date = state.getDate();
//                
//                
//                //angle between sun earth and satellite
//                final PVCoordinates earthPV    = CelestialBodyFactory.getEarth().getPVCoordinates(date, eme2000);
//                final PVCoordinates sunPV    = sun.getPVCoordinates(date, eme2000);
//                final PVCoordinates satPV = state.getPVCoordinates(eme2000);
//                final PVCoordinates earthToSatPV  = new PVCoordinates(earthPV, satPV);
//                final Vector3D      earthToSat = earthToSatPV.getPosition();
//                final PVCoordinates earthToSunPV  = new PVCoordinates(earthPV, sunPV);
//                final Vector3D      earthToSun = earthToSunPV.getPosition();
//                final double AngleBetweenSatEarthSun = Vector3D.angle(earthToSun, earthToSat);
//                
//                final Transform inertialToLof = Lof.transformFromInertial(state.getDate(), satPV);
//                
////                Vector3D srpAccVectorInert = radiationPressureAcceleration(state.getDate(),eme2000 , satPV.getPosition(),state.getAttitude().getRotation(), state.getMass(), flux, parameters);
//                
//                //find pointing vector from sat to sun
//                final PVCoordinates pointing  = new PVCoordinates(satPV, sunPV);
//                final Vector3D      pointingP = pointing.getPosition();
//                
//                //find vector normal to Solar Array
//                final Attitude CurrAttinFrame = state.getAttitude();
//                Rotation rotInframe = CurrAttinFrame.getRotation();
//                Attitude CurrAttinInertial = new Attitude(state.getDate(), eme2000, 
//                		rotInframe.compose(frameToinertial.getRotation(), RotationConvention.VECTOR_OPERATOR),
//                		rotInframe.applyTo(frameToinertial.getRotationRate()),
//                		rotInframe.applyTo(frameToinertial.getRotationAcceleration())
//                		);
//                
//                
//                final double alignmentAngle = crossSun.g(state);
//                
////        		final Vector3D normal = spacecraft.getNormal(date, eme2000, satPV.getPosition(), CurrAttinInertial.getRotation());//coords now from inertialFrame
//                
//                
//                final DecimalFormatSymbols angleDegree = new DecimalFormatSymbols(Locale.US);
//                angleDegree.setDecimalSeparator('\u00b0');
//                final DecimalFormat ad = new DecimalFormat(" 00.00;-00.0", angleDegree);
//            	
//            	output.add(String.format(Locale.US, "%s a = %12.3f m, e = %11.9f, i = %8.4f, m = %8.3f kg, RotX = %s, RotY = %s, RotZ = %s, AngleBetweenSatEarthSun = %s, srpAccVector = %s, Alignment = %s %n", // pointToSunVec = %s, normalToArr = %s,
//                        state.getDate(), 
//                        state.getA(), 
//                        state.getE(), 
//                        state.getI(),
//                        state.getMass(),
//                        ad.format(FastMath.toDegrees(anglesV[0])), 
//                        ad.format(FastMath.toDegrees(anglesV[1])), 
//                        ad.format(FastMath.toDegrees(anglesV[2])),
//                        ad.format(FastMath.toDegrees(AngleBetweenSatEarthSun)),
////                        inertialToLof.transformVector(pointingP.normalize()).toString(),
////            			inertialToLof.transformVector(normal.normalize()).toString(),
//                        inertialToLof.transformVector(srpAccVectorInert).toString(),
//                        ad.format(FastMath.toDegrees(alignmentAngle))
//                        ));
            	AbsoluteDate currabsdate = state.getDate();
            	
            	ArrayList<String> line = new ArrayList<String>();
            	line.add(currabsdate.toString());
            	line.add(String.valueOf(state.getA()));
            	line.add(String.valueOf(state.getE()));
            	line.add(String.valueOf(state.getI()));
            	toFile.add(line);
            	
            });

            // Propagate from the initial date for the fixed duration
            final SpacecraftState finalState = propagator.propagate(initialDate.shiftedBy(initialOrbit.getKeplerianPeriod() * 100000));
//            initialDate.shiftedBy(initialOrbit.getKeplerianPeriod() * 100000)
            Path currentpath = Paths.get(System.getProperty("user.home"));
            Path filepath = Paths.get(currentpath.toString(), "/Documents/projects/Space Debris De-orbiter");
            
            FileWriter csvWriter = new FileWriter(filepath + "/data100by100-100kg.csv");
            
            csvWriter.append("Date");
            csvWriter.append(",");
            csvWriter.append("SemiMajorAxis");
            csvWriter.append(",");
            csvWriter.append("Eccentricity");
            csvWriter.append(",");
            csvWriter.append("Inclination");
            csvWriter.append("\n");
            
            for( ArrayList<String> line : toFile) {
            	csvWriter.append(String.join(",", line));
                csvWriter.append("\n");
            }
            
            csvWriter.flush();
            csvWriter.close();
            
            // we print the lines according to lexicographic order, which is chronological order here
            // to make sure out of orders calls between step handler and event handlers don't mess things up
//            for (final String line : output) {
//                System.out.println(line);
//            }

            final Orbit finalOrbit = finalState.getOrbit();
            final double fA = finalOrbit.getA();
            final double fE = finalOrbit.getE();
            final double iA = initialOrbit.getA();
            final double iE = initialOrbit.getE();
            final double iPE = iA*(1-iE);
            final double fPE = fA*(1-fE);
            
            
            System.out.println("Propagation ended at " + finalState.getDate());
            System.out.println("Period " + finalState.getOrbit().getKeplerianPeriod());
            System.out.println("Time elapsed " + finalState.getDate().durationFrom(initialDate)/60 + " min");
            System.out.println("Time elapsed in years " + finalState.getDate().durationFrom(initialDate)/(60*60*24*365) + " years");
            System.out.println("Initial Orbit:" + initialOrbit.toString());
            System.out.println("Final Orbit:" + finalState.getOrbit().toString());
            System.out.println("Initial Periapsis:" + iPE + " m" );
            System.out.println("Final Periapsis: " +  fPE + " m");
            System.out.println("Diff in PE: " + (iPE - fPE)/1000 + " km");
            System.out.println("Difference in semi-major axis: " + (finalState.getOrbit().getA() - initialOrbit.getA()));

        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
        } catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
    }

}
