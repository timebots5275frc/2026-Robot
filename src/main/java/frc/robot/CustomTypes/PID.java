package frc.robot.CustomTypes;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class PID {
    public double p,i,d,kS,kV,kA,kG,iz;
    public PID(double p, double i, double d, double kS) {this.p=p; this.i=i; this.kS=kS; this.d=d;}
    public PID(double p, double i, double d, double iz, double kS) {this.p=p; this.i=i; this.d=d; this.iz=iz; this.kS=kS;}
    public PID(double p, double i, double d, double kS, double kV, double kA, double kG){this.p=p; this.i=i; this.d=d; this.kS=kS; this.kV=kV; this.kA=kA; this.kG=kG;}
    public PID() {this(0,0,0,0);}
    public PID(double p, double i, double d) {this(p,i,d,0);}
    public PID(PID cpy) {this(cpy.p, cpy.i, cpy.d);}

    public final IdleMode _imode_default = IdleMode.kBrake;
    public final PersistMode  _pmode_default = PersistMode.kNoPersistParameters;
    public final ResetMode _rmode_default = ResetMode.kResetSafeParameters;
    private IdleMode _imode;
    private PersistMode _pmode;
    private ResetMode _rmode;
    private int stallLimit = 20; //Defaults
    private int freeLimit = 20;  //Defaults

    public SparkMaxConfig setSparkMaxPID(SparkMax spm, ResetMode rm, PersistMode pm)
    {
      this._rmode=rm;
      this._pmode=pm;
      return setPIDBase(spm);
    }
    public SparkMaxConfig setSparkMaxPID(SparkMax spm) {return setPIDBase(spm);}
    public SparkMaxConfig setSparkMaxPID(SparkMax spm, IdleMode im) {
      this._imode = im;
      return setPIDBase(spm);
    }
    public SparkMaxConfig setSparkMaxPID(SparkMax spm, ResetMode rm, PersistMode pm, IdleMode im)
    {
      this._imode=im;
      this._pmode=pm;
      this._rmode=rm;
      return setPIDBase(spm);
    }
    public SparkFlexConfig setSparkFlexPID(SparkFlex spm) {return setPIDBaseFlex(spm);}

    public SparkFlexConfig setSparkFlexPID(SparkFlex spm, IdleMode im) {
      this._imode = im;
      return setPIDBaseFlex(spm);
    }
    public SparkFlexConfig setSparkFlexPID(SparkFlex spm, ResetMode rm, PersistMode pm)
    {
      this._rmode=rm;
      this._pmode=pm;
      return setPIDBaseFlex(spm);
    }
    public SparkFlexConfig setSparkFlexPID(SparkFlex spm, ResetMode rm, PersistMode pm, IdleMode im)
    {
      this._imode=im;
      this._pmode=pm;
      this._rmode=rm;
      return setPIDBaseFlex(spm);
    }
    private SparkMaxConfig setPIDBase(SparkMax sp) {
      ClosedLoopConfig c = new ClosedLoopConfig();
      c.pidf(p,i,d,kS);
      c.iZone(iz);
      SparkMaxConfig sc = new SparkMaxConfig();
      sc.apply(c);
      sc.smartCurrentLimit(stallLimit,freeLimit);
      sc.idleMode( (_imode!= null)?(_imode):(_imode_default) );
      sp.configure(sc, (_rmode != null)?(_rmode):(_rmode_default), (_pmode!= null)?(_pmode):(_pmode_default));
      return sc;
    }

    private SparkFlexConfig setPIDBaseFlex(SparkFlex sp) {
      ClosedLoopConfig c = new ClosedLoopConfig();
      c.pidf(p,i,d,kS);
      c.iZone(iz);
      SparkFlexConfig sc = new SparkFlexConfig();
      sc.smartCurrentLimit(stallLimit,freeLimit);
      sc.apply(c);
      sc.idleMode( (_imode!= null)?(_imode):(_imode_default) );
      sp.configure(sc, (_rmode != null)?(_rmode):(_rmode_default), (_pmode!= null)?(_pmode):(_pmode_default));
      return sc;
    }

    public void setFreeLimit(int freeLimit) {
      this.freeLimit = freeLimit;
    }
     public void setStallLimit(int stallLimit) {
      this.stallLimit = stallLimit;
    }
}