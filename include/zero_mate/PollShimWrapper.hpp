#ifdef _WIN32
    #include <winsock2.h>
using wake_fd_t = SOCKET;
#else
    #include <unistd.h>
using wake_fd_t = int;
#endif

#ifdef _WIN32
    #include <winsock2.h>
typedef SOCKET wake_fd_t;
typedef WSAPOLLFD poll_fd_t;
    #define poll_sockets(fds, n, ms) WSAPoll(fds, n, ms)
#else
    #include <poll.h>
using wake_fd_t = int;
using poll_fd_t = struct pollfd;
    #define poll_sockets(fds, n, ms) poll(fds, n, ms)
#endif

class WakePipe
{
public:
    WakePipe()
    {
#ifdef _WIN32
        SOCKET listener = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr{ };
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        bind(listener, (sockaddr*)&addr, sizeof(addr));
        listen(listener, 1);

        int len = sizeof(addr);
        getsockname(listener, (sockaddr*)&addr, &len);

        fds_[1] = socket(AF_INET, SOCK_STREAM, 0);
        connect(fds_[1], (sockaddr*)&addr, sizeof(addr));
        fds_[0] = accept(listener, nullptr, nullptr);
        closesocket(listener);
#else
        pipe(fds_);
#endif
    }

    ~WakePipe()
    {
#ifdef _WIN32
        closesocket(fds_[0]);
        closesocket(fds_[1]);
#else
        close(fds_[0]);
        close(fds_[1]);
#endif
    }

    [[nodiscard]] wake_fd_t read_fd() const
    {
        return fds_[0];
    }
    [[nodiscard]] wake_fd_t write_fd() const
    {
        return fds_[1];
    }

    void signal()
    {
#ifdef _WIN32
        send(fds_[1], "\x00", 1, 0);
#else
        write(fds_[1], "\x00", 1);
#endif
    }

    // Non-copyable, moveable if needed
    WakePipe(const WakePipe&) = delete;
    WakePipe& operator=(const WakePipe&) = delete;

    WakePipe(WakePipe&&) = default;
    WakePipe& operator=(WakePipe&&) = default;

private:
    wake_fd_t fds_[2]{ };
};
