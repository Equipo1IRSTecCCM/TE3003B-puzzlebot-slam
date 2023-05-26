# Puzzlebot to robonet

## STEP 1   
Check if the Wifi and Ethernet port are connected in the puzzlebot trhough a terminal connect via SSH

    nmcli dev status

## STEP 2
Connect your computer to the modem using the following credentials:

    SSID = robonet

    Password = sam12345

## STEP 3
Connect the puzzlebot to the modem via ethernet and find it's IP by going to the following address in a browser in your computer:

    192.168.0.1

    Password = roborrobos

Once you're in, go to the tab Advanced > Network > DHCP Server

There, you can find at the bottom of the page the assigned IPs, find the one assigned to puzzlebot-desktop and save it for later

## STEP 4

Connect to the puzzlebot using SSH running the following command:

    ssh puzzlebot@($ puzzlebot-desktop IP)

## STEP 5

Check that the wifi is still on

    nmcli radio wifi

If it says disabled, run the following command:

    nmcli radio wifi on

## STEP 6

Turn off the hotspot with:

    sudo nmcli con down id Hotspot

## STEP 7

Connect to "robonet" with:

    sudo nmcli dev wifi connect robonet password "sam12345"

If it's not the first time you're connecting to the network use:

    sudo nmcli con up robonet

## STEP 8

Disconnect the puzzlebot from the ethernet cable and you can connect trhough the wifi, repeat the steps from STEP 3 and find the latest IP assigned to puzzlebot-desktop. That's the IP to communicate with the puzzlebot.