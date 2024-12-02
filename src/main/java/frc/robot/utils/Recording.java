package frc.robot.utils;

import java.util.HashMap;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Recording implements Sendable {

  static Recording instance;
  String status = "";
  HashMap<String, Integer> statusCode = new HashMap<String, Integer>();

  Recording() {
    instance = this;

    statusCode.put("START", 0);
    statusCode.put("PAUSE", 1);
    statusCode.put("STOP", 2);
    statusCode.put("FLAG", 3);

    LogManager.addEntry(status, this::getStatusCode);
  }

  private int getStatusCode() {
    Integer x = statusCode.get(status);
    if (x != null)
      return x;
    return -1;
  }

  public static Recording getInstance() {
    if (instance != null) {
      return instance;
    }
    
    return new Recording();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Recording");
    
    builder.addStringProperty("Status", ()-> status, newStatus -> status = newStatus);
  }
}
