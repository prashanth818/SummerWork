public class PythonInterface{
  ArmController arm_controller;

  public PythonInterface() {
      arm_controller = new ArmController();
    }

    public ArmController getArmController() {
        return arm_controller;
    }

  public static void main(String[] args) {
    System.out.println("hello world");
    GatewayServer gatewayServer = new GatewayServer(new ArmController(), 12345);
    System.out.println("Gateway Server Started");
    gatewayServer.start();
  }
}
