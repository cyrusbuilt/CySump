#!/bin/bash

echo
cat >> /dev/stdout << EOF
This tool will generate a Certificate Authority (CA), which
is then used to generate and sign server certificates and fingerprint
used for encrytped 2-way communication between CYGARAGE and an MQTT
broker over TLS. Once the necessary certs and SHA256 fingerprint
are generated, the necessary files will be placed in the 'data'
directory so that the SPIFFS image can then be flashed onto the
CYAGARAGE device, and therefore loaded during boot. The server
keys will then need to be installed and configured on the MQTT
broker.
EOF

echo
if ! read -n1 -rsp $'Press any key to continue or CTRL+C to exit.\n'; then
    exit 1
fi

# We can't do a damn thing without the openssl CLI tool.
echo
TOOL="$( which openssl )"
if [[ -z ${TOOL} ]]; then
  echo "ERROR: openssl not installed."
  exit 1  
fi

# We need to clean up any existing certs first.
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"
CERT_DIR=${SCRIPT_DIR}/certs
if [ -d ${CERT_DIR} ]; then
    rm -rf ${CERT_DIR}
fi

mkdir ${CERT_DIR}
if [ -f ${CERT_DIR}/../../data/ca.crt ]; then
    rm -f ${CERT_DIR}/../../data/ca.crt
fi

if [ -f ${CERT_DIR}/../../data/mqtt.fpn ]; then
    rm -f ${CERT_DIR}/../../data/mqtt.fpn
fi

read -rp $'Enter host name for server certificate: ' hostname
read -rp $'Enter host IP for server certificate: ' host_ip

# Generate the config file used for signing. The most important piece of this
# config is the v3_req extension which enables the X509 v3 requests (which
# is not enabled by default).
echo
echo "Generating config file..."
cat <<EOF > ${CERT_DIR}/openssl.conf
[req]
distinguished_name = req_distinguished_name
req_extensions = v3_req

[req_distinguished_name]
countryName = Country Name (2 letter code)
countryName_default = US
stateOrProvinceName = State or Province Name (full name)
localityName = Locality Name (eq, city)
organizationalUnitName = Organizational Unit Name (eg, section)
commonName = Organization Name (eg, company)
commonNam_max = 64

[ v3_req ]
# Extensions to add to a certificate request
basicConstraints = CA:FALSE
keyUsage = nonRepudiation, digitalSignature, keyEncipherment
subjectAltName = @alt_names

[alt_names]
DNS.1 = localhost
DNS.2 = ${hostname}
IP.1 = 127.0.0.1
IP.2 = ${host_ip}
EOF

echo
echo "Generating certificate authority key..."
openssl ecparam \
    -name secp521r1 \
    -genkey \
    -noout \
    -out ${CERT_DIR}/ca.key.pem

echo
echo "Generating certificate authority cert..."
openssl req \
    -new \
    -x509 \
    -days 3650 \
    -key ${CERT_DIR}/ca.key.pem \
    -config ${CERT_DIR}/openssl.conf \
    -out ${CERT_DIR}/ca.crt.pem \
    -sha256

echo
echo "Generating server certificate key..."
openssl ecparam \
    -name secp521r1 \
    -genkey \
    -noout \
    -out ${CERT_DIR}/${hostname}.key.pem

echo
echo "Generating server certificate signing request..."
openssl req -new \
    -out ${CERT_DIR}/${hostname}.csr.pem \
    -key ${CERT_DIR}/${hostname}.key.pem \
    -config ${CERT_DIR}/openssl.conf \
    -sha256

echo
echo "Signing server certificate..."
openssl x509 -req \
    -days 3650 \
    -in ${CERT_DIR}/${hostname}.csr.pem \
    -CA ${CERT_DIR}/ca.crt.pem \
    -CAkey ${CERT_DIR}/ca.key.pem \
    -CAcreateserial \
    -out ${CERT_DIR}/${hostname}.crt.pem \
    -extensions v3_req \
    -extfile ${CERT_DIR}/openssl.conf \
    -sha256

echo
echo "Generating server fingerprint..."
openssl x509 \
    -in ${CERT_DIR}/${hostname}.crt.pem \
    -fingerprint \
    -noout > ${CERT_DIR}/${hostname}.fp
cat ${CERT_DIR}/${hostname}.fp

echo
echo "Copying server certs to SPIFFS image data directory..."
# SPIFFS requires the files be named in 8.3 format (its essentially a FAT filesystem).
cp -f ${CERT_DIR}/ca.crt.pem ${CERT_DIR}/../../data/ca.crt
cp -f ${CERT_DIR}/${hostname}.fp ${CERT_DIR}/../../data/mqtt.fpn

echo
echo "IMPORTANT: You will need to install the server certificates"
echo "for your MQTT broker before connecting the client."
echo "The necessary files will be as follows:"
echo
echo "${CERT_DIR}/${hostname}.crt.pem"
echo "${CERT_DIR}/${hostname}.key.pem"
echo "${CERT_DIR}/ca.crt.pem"
echo
echo "NOTE: These certificates will expire in 1 Year from today."
echo
echo "Done"