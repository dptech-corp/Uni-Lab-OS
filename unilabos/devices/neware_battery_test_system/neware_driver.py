import socket
END_MARKS = [b"\r\n#\r\n", b"</bts>"]  # 读到任一标志即可判定完整响应

def build_start_command(devid, subdevid, chlid, CoinID,
                        ip_in_xml="127.0.0.1",
                        devtype:int=27,
                        recipe_path:str=f"D:\\HHM_test\\A001.xml",
                        backup_dir:str=f"D:\\HHM_test\\backup") -> str:
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<bts version="1.0">',
        '  <cmd>start</cmd>',
        '  <list count="1">',
        f'    <start ip="{ip_in_xml}" devtype="{devtype}" devid="{devid}" subdevid="{subdevid}" chlid="{chlid}" barcode="{CoinID}">{recipe_path}</start>',
        f'    <backup backupdir="{backup_dir}" remotedir="" filenametype="1" customfilename="" createdirbydate="0" filetype="0" backupontime="1" backupontimeinterval="1" backupfree="0" />',
        '  </list>',
        '</bts>',
    ]
    # TCP 模式：请求必须以 #\r\n 结束（协议要求）
    return "\r\n".join(lines) + "\r\n#\r\n"

def recv_until_marks(sock: socket.socket, timeout=60):
    sock.settimeout(timeout)  # 上限给足，协议允许到 30s:contentReference[oaicite:2]{index=2}
    buf = bytearray()
    while True:
        chunk = sock.recv(8192)
        if not chunk:
            break
        buf += chunk
        # 读到结束标志就停，避免等对端断开
        for m in END_MARKS:
            if m in buf:
                return bytes(buf)
        # 保险：读到完整 XML 结束标签也停
        if b"</bts>" in buf:
            return bytes(buf)
    return bytes(buf)

def start_test(ip="127.0.0.1", port=502, devid=3, subdevid=2, chlid=1, CoinID="A001", recipe_path=f"D:\\HHM_test\\A001.xml", backup_dir=f"D:\\HHM_test\\backup"):
    xml_cmd = build_start_command(devid=devid, subdevid=subdevid, chlid=chlid, CoinID=CoinID, recipe_path=recipe_path, backup_dir=backup_dir)
    #print(xml_cmd)
    with socket.create_connection((ip, port), timeout=60) as s:
        s.sendall(xml_cmd.encode("utf-8"))
        data = recv_until_marks(s, timeout=60)
    return data.decode("utf-8", errors="replace")

if __name__ == "__main__":
    resp = start_test(ip="127.0.0.1", port=502, devid=4, subdevid=10, chlid=1, CoinID="A001", recipe_path=f"D:\\HHM_test\\A001.xml", backup_dir=f"D:\\HHM_test\\backup")
    print(resp)
