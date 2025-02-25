#!/bin/bash

# Define the path for the output XML file
output_xml="$HOME/AUSPEX/utils/dds_scripts/fastrtps_dds_tailscale/fastrtps_dds_setup.xml"

# Set debug mode (true or false)
debug=false

# Function to handle debug output
debug_echo() {
  if $debug; then
    echo "$@"
  fi
}

# Check for a command-line argument
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 [MULTI|LOCAL]"
    exit 1
fi

mode="$1"

if [[ "$mode" != "MULTI" && "$mode" != "LOCAL" ]]; then
    echo "Error: Invalid argument. Use 'MULTI' or 'LOCAL'."
    exit 1
fi



# Fetch tailscale status
tailscale_output=$(tailscale status)

# Debug output for tailscale status
debug_echo "Debug: Tailscale status output:"
debug_echo "$tailscale_output"
debug_echo "-----------------------------------"

# Extract only Tailscale IP addresses
if [ "$mode" == "MULTI" ]; then
    ip_addresses=$(echo "$tailscale_output" | awk '/100\.[0-9]+\.[0-9]+\.[0-9]+/ && $NF != "offline" {print $1}')
else
    ip_addresses="" # In LOCAL mode, we don't use Tailscale IPs
fi

# Debug output for captured IP addresses
debug_echo "Debug: Captured IP addresses:"
debug_echo "$ip_addresses"
debug_echo "-----------------------------------"

# XML parts before and after IP addresses
xml_header='<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
         <transport_descriptor>
            <transport_id>TransportId1</transport_id>
            <type>UDPv4</type> 
            <maxInitialPeersRange>256</maxInitialPeersRange> 
        </transport_descriptor>
    </transport_descriptors>


    <participant profile_name="TailscaleProfile" is_default_profile="true">
        <rtps>
            <useBuiltinTransports>true</useBuiltinTransports>
            <userTransports>
                <transport_id>TransportId1</transport_id>
            </userTransports>
            <builtin>
                <initialPeersList>
' # New line is important!

xml_footer='                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>'

# Initialize locator entries variable
locator_entries=""

# Loop through each IP and construct the locator XML part
if [ "$mode" == "MULTI" ]; then
    for ip in $ip_addresses; do
        locator_entries+="                    <locator>
                        <udpv4>
                            <address>$ip</address>
                        </udpv4>
                    </locator>
"
    done
fi

locator_entries+="                    <locator>
                        <udpv4>
                            <address>127.0.0.1</address>
                        </udpv4>
                    </locator>
"

# Check if any IP addresses were found
if [ "$mode" == "MULTI" ] && [ -z "$ip_addresses" ]; then
    echo "Warning by ~/AUSPEX/utils/dds_scripts/fastrtps_dds_tailscale/update_fastdds_xml.sh: TAILSCALE NOT AVAILABLE!"
else
    ip_count=$(echo "$ip_addresses" | wc -w)
    debug_echo "$ip_count valid IP addresses found."

    # Combine all parts to form the final XML content
    final_xml_content="$xml_header$locator_entries$xml_footer"

    # Write the final XML content to the specified file
    echo "$final_xml_content" > "$output_xml"

    debug_echo "fastrtps_dds_setup.xml file updated successfully."
fi