openssl genrsa -out private.pem -aes128 -passout pass:$1
openssl rsa -in private.pem -outform PEM -pubout -out public.pem -passin pass:$1