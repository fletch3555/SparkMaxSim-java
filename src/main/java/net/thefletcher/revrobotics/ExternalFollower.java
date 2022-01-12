package net.thefletcher.revrobotics;

public class ExternalFollower {
  public final int arbId;
  public final int configId;

  public static final ExternalFollower kFollowerDisabled = new ExternalFollower(0, 0);
  public static final ExternalFollower kFollowerSparkMax = new ExternalFollower(0x2051800, 26);
  public static final ExternalFollower kFollowerPhoenix = new ExternalFollower(0x2040080, 27);

  public ExternalFollower(int arbId, int configId) {
    this.arbId = arbId;
    this.configId = configId;
  }
}