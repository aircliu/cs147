#ifndef SECRETS_H
#define SECRETS_H

// AWS IoT certificates
const char* DEVICE_CERT = R"EOF(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)EOF";

const char* PRIVATE_KEY = R"EOF(
-----BEGIN RSA PRIVATE KEY-----

-----END RSA PRIVATE KEY-----
)EOF";

const char* ROOT_CA = R"EOF(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)EOF";

const char* AWS_IOT_ENDPOINT = "-ats.iot.us-east-2.amazonaws.com";

#endif