package Wifi;

import java.util.Map;



public class WiFiData {
	/**
	 * The WiFi Data Class
	 * 
	 * @author Noah Levine
	 * @version 1.0
	 * @since 2017-03-24  	
	 */
	private static final String SERVER_IP = "192.168.2.11";
	private static final int TEAM_NUMBER = 5;
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
	private Map wifiData;
	
	public boolean offense = false;
	public boolean defense = false;
	
	public int corner;
	public int w1;
	public int w2;
	public int bx;
	public int by;
	public int d1;
	
	public String orientation;
	
	/**
	 * Constructor for the WiFiData	
	 */
	public WiFiData() {
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
		try {
			wifiData = conn.getData();
			
			int fwdTeam = ((Long) wifiData.get("FWD_TEAM")).intValue();
			if (fwdTeam==5){
				
				 offense = true;
				corner= ((Long) wifiData.get("FWD_CORNER")).intValue();
			}
			
			int defTeam = ((Long) wifiData.get("DEF_TEAM")).intValue();
			if (defTeam == 5) {
				
				defense = true;
				corner= ((Long) wifiData.get("DEF_CORNER")).intValue();
			}
			
			if (offense || defense) {
				w1 = ((Long) wifiData.get("w1")).intValue();
				w2 = ((Long) wifiData.get("w2")).intValue();
				bx= ((Long) wifiData.get("bx")).intValue();
				by= ((Long) wifiData.get("by")).intValue();
				orientation = (String) wifiData.get("omega");
				d1= ((Long) wifiData.get("d1")).intValue();
			}
			
		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}
		

	}
	
	


}
