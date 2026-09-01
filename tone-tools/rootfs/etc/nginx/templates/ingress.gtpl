events {
    worker_connections 16;
}
http {
    include /etc/nginx/mime.types;
    default_type application/octet-stream;
    sendfile on;
    keepalive_timeout 65;
    access_log /dev/stdout;
    error_log /dev/stderr info;

    server {
        listen 8099 default_server;
        server_name _;

        root /www;
        index index.html;

        location / {
            try_files $uri $uri/ /index.html;
            add_header Cache-Control "no-cache" always;
            add_header X-Frame-Options "SAMEORIGIN" always;
        }

        location /wasm/ {
            alias /www/wasm/;
            add_header Cache-Control "public, max-age=86400" always;
            add_header Access-Control-Allow-Origin "*" always;
            types {
                application/wasm wasm;
                application/javascript js;
            }
        }

        location /js/ {
            alias /www/js/;
            add_header Cache-Control "public, max-age=86400" always;
        }

        location /api/ {
            proxy_pass http://127.0.0.1:8098/;
            proxy_set_header Host $host;
            proxy_set_header X-Real-IP $remote_addr;
            proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
            proxy_set_header X-Forwarded-Proto $scheme;
            proxy_read_timeout 300s;
            proxy_send_timeout 300s;
            client_max_body_size 50m;
        }

        location /health {
            access_log off;
            return 200 "ok\n";
            add_header Content-Type text/plain;
        }
    }
}
